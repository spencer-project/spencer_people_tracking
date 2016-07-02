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
#include <angles/angles.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/polygon_occlusion_manager.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/base/lap.h>

#define foreach BOOST_FOREACH

namespace srl_nnt {

PolygonOcclusionManager::PolygonOcclusionManager() :
  m_numberLaserScanners(0), m_visualizationEnabled(false), m_detectionProbabilityOccluded(0.2), m_detectionProbabilityVisible(0.9)
{
}

void PolygonOcclusionManager::initializeOcclusionManager(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle)
{
    // Save nodehandles in members
    m_nodeHandle = nodeHandle;
    m_privateNodeHandle = privateNodeHandle;

    // Initialize sampler
    m_visualizationEnabled = Params::get<bool>("occlusion_manager_visualization_enabled", false);
    m_addtionalTimeFactor = Params::get<double>("occlusion_manager_time_uncertainty_factor",1.2);
    m_occlusionMaxRange = Params::get<double>("occlusion_manager_max_range",60);
    m_neighborPolygonWidth = Params::get<double>("occlusion_neigbor_polygon_width",0.2);
    m_selfOcclusionDistance = Params::get<double>("occlusion_self_occlusion_distance",0.25);
    m_allowedDurationForReappearance = Params::get<double>("occlusion_allowed_duration_for_reappearance",2.0);
    m_scaleUpdateWithCost = Params::get<bool>("occlusion_geodesics_scale_reassignment",false);
    m_maximumSyncSlop = Params::get<double>("occlusion_manager_maximum_sync_slop",0.04);
    m_minNumberMatches = Params::get<int>("occlusion_manager_minimum_matches",0);
    m_minAbsoluteVelocity = Params::get<int>("occlusion_manager_minimum_absolute_velocity",0.1);
    m_numberOfReappereadFramesToZeroVelocity = Params::get<int>("occlusion_manager_number_of_reappeared_frames_to_zero_velocity",5);

    // Get number of laser scanners and subscribe to the corresponding topics
    int laserSubscriptions = Params::get<int>("occlusion_manager_number_lasers",1);
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

    m_visualizationPublisher = m_nodeHandle.advertise<visualization_msgs::MarkerArray>("occlusion_markers", 5);

    m_MAX_MISSES_BEFORE_DELETION = Params::get<int>("max_misses_before_deletion", 8);
    m_MAX_MISSES_BEFORE_DELETION_OF_MATURE_TRACK = Params::get<int>("max_misses_before_deletion_of_mature_track", 15);
    m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES = Params::get<int>("track_is_mature_after_total_num_matches", 100);

    m_occlusion_geodesics_inertia_variance = Params::get<double>("occlusion_geodesics_inertia_variance", 0.1);
    m_occlusion_geodesics_motion_variance = Params::get<double>("occlusion_geodesics_motion_variance", 1.0);
    m_occlusion_geodesics_plausibility_cut_off = Params::get<double>("occlusion_geodesics_plausibility_cut_off", 0.01);
    m_occlusion_geodesics_total_cut_off = Params::get<double>("occlusion_geodesics_total_cut_off", 1e-3);
    m_occlusion_geodesics_use_detection_probability = Params::get<int>("occlusion_geodesics_use_detection_probability", 1);
    m_occlusion_geodesics_use_inverted_plausibility = Params::get<int>("occlusion_geodesics_use_inverted_plausibility", 1);
    m_occlusion_geodesics_use_plausibility = Params::get<int>("occlusion_geodesics_use_plausibility", 1);
    m_occlusion_geodesics_use_duration_cost = Params::get<int>("occlusion_geodesics_use_duration_cost", 1);

    ROS_INFO_STREAM("#### Polygon Occlusion Manager configured as follows: #####\n "
            << "occlusion_manager_visualization_enabled:" << m_visualizationEnabled << "\n"
            << "occlusion_manager_time_uncertainty_factor:" << m_addtionalTimeFactor << "\n"
            << "occlusion_manager_max_range:" << m_occlusionMaxRange << "\n"
            << "occlusion_manager_number_lasers:" << laserSubscriptions << "\n"
            << "occlusion_geodesics_inertia_variance:" << m_occlusion_geodesics_inertia_variance << "\n"
            << "occlusion_geodesics_motion_variance:" << m_occlusion_geodesics_motion_variance << "\n"
            << "occlusion_geodesics_plausibility_cut_off:" << m_occlusion_geodesics_plausibility_cut_off << "\n"
            << "occlusion_geodesics_total_cut_off:" << m_occlusion_geodesics_total_cut_off << "\n"

    );
}

void PolygonOcclusionManager::deleteOccludedTracks(Tracks& tracks, const ros::Time& time)
{
    ROS_DEBUG("Deleting obsolete tracks");

    size_t numDeletedTracks = 0;
    std::vector<Tracks::iterator> tracksToDelete;
    for(Tracks::iterator trackIt = tracks.begin(); trackIt != tracks.end(); ++trackIt) {
        Track::Ptr track = *trackIt;
        switch (track->trackStatus) {
            case Track::MATCHED:
                if(m_likelyOccludedTracks.find(track->id) != m_likelyOccludedTracks.end())
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was likely to be occluded but matched anyway!");
                }
                else if(m_occludedTracks.find(track->id) != m_occludedTracks.end())
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was occluded but matched anyway!");
                    m_occludedTracks.erase(track->id);
                }
                break;
            case Track::MISSED:
            {
                if(m_likelyOccludedTracks.find(track->id) != m_likelyOccludedTracks.end())
                {
                    double secsToOcclusion = ros::Duration(m_likelyOccludedTracks.at(track->id)->occlusionBeginTime - time).toSec();
                    if(secsToOcclusion >= 0)
                    {
                        ROS_DEBUG_STREAM("Track " << track->id << " was missed due to a likely occlusion! Occlusion should start in " << secsToOcclusion << "sec");
                    }
                    else
                    {
                        ROS_DEBUG_STREAM("Track " << track->id << " should already be occluded " << secsToOcclusion << "seconds ago");
                        m_likelyOccludedTracks.erase(track->id);
                        tracksToDelete.push_back(trackIt);
                        numDeletedTracks++;
                    }
                }
                else
                {
                    // Check if the track is considered as "mature", i.e. it has been there for a long time already.
                    const bool trackIsMature = track->numberOfTotalMatches >= m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES;

                    // Check if the track hasn't been seen for too long.
                    const int missedFrameLimit = trackIsMature ? m_MAX_MISSES_BEFORE_DELETION_OF_MATURE_TRACK : m_MAX_MISSES_BEFORE_DELETION;
                    if(track->numberOfConsecutiveMisses > missedFrameLimit)
                    {
                        if(m_occludedTracks.find(track->id) != m_occludedTracks.end())
                        {
                            ROS_DEBUG_STREAM("Track " << track->id << " is contained in occlusion map and reached missdetection limit!");
                            m_occludedTracks.erase(track->id);
                        }
                        else
                        {
                            ROS_DEBUG_STREAM("Track " << track->id << " is not contained in any occlusion map therefore basic missdetection handling");
                        }
                        tracksToDelete.push_back(trackIt);
                        ROS_INFO_STREAM("Deleting track with id " << track->id << " with  " << track->numberOfConsecutiveMisses << " consecutive misses." );
                        numDeletedTracks++;
                    }
                    else
                    {
                        ROS_DEBUG_STREAM("Keeping track with id " << track->id << " with  " << track->numberOfConsecutiveMisses << " consecutive misses." );
                    }
                }
                break;
            }
            case Track::OCCLUDED:
                if(m_likelyOccludedTracks.find(track->id) != m_likelyOccludedTracks.end())
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was missed due to a likely occlusion!");
                    m_likelyOccludedTracks.erase(track->id);
                }
                else if(m_occludedTracks.find(track->id) != m_occludedTracks.end())
                {
                    double timeSinceOcclusion = ros::Duration(time - m_occludedTracks.at(track->id)->occlusionEndTime).toSec();
                    if(timeSinceOcclusion > m_allowedDurationForReappearance)
                    {
                        ROS_WARN_STREAM("Track " << track->id << " was occluded and but could not be matched for " << timeSinceOcclusion << "seconds. We delete the track!");
                        m_occludedTracks.erase(track->id);
                        tracksToDelete.push_back(trackIt);
                        numDeletedTracks++;
                    }
                    else if(timeSinceOcclusion < 0)
                    {
                        ROS_DEBUG_STREAM("Track " << track->id << " is still occluded for " << fabs(timeSinceOcclusion) << "seconds.");

                    }
                    else
                    {
                        ROS_DEBUG_STREAM("Track " << track->id << " was occluded and but could not be matched for " << timeSinceOcclusion << "seconds. We wait!");
                    }
                }

                break;

            default:
                break;
        }
    }
    for(int i = tracksToDelete.size()-1; i >= 0; i--)
    {
        tracks.erase(tracksToDelete.at(i));
    }
    if(numDeletedTracks) ROS_WARN("%zu track(s) have been deleted!", numDeletedTracks);

    publishMarkerArray();
}

void PolygonOcclusionManager::subscribeToLaser(const std::string laserTopic, const std::string laserSegmentationTopic)
{
    int queue_size = Params::get<int>("occlusion_manager_subscriber_queue_size",35);
    int circular_buffer_size = Params::get<int>("occlusion_manager_sync_buffer_size",10);

    ROS_INFO_STREAM("Subscribing to laser with topic " << laserTopic << " and segmentation with topic " << laserSegmentationTopic);
    
    // Subscribers
    LaserscanSubscriber_t subLaser ( new message_filters::Subscriber<sensor_msgs::LaserScan>(m_nodeHandle, laserTopic, queue_size, ros::TransportHints(), &m_callbackQueue) );
    m_laserscanSubscribers.push_back( subLaser);
    SegmentationSubscriber_t subSegmentation (new message_filters::Subscriber<srl_laser_segmentation::LaserscanSegmentation>(m_nodeHandle, laserSegmentationTopic, queue_size, ros::TransportHints(), &m_callbackQueue) );
    m_segmentationSubscribers.push_back(subSegmentation );

    InputSynchronizer_t synchronizer (new Synchronizer_t(SyncPolicy_t(queue_size), *(m_laserscanSubscribers.back()), *(m_segmentationSubscribers.back()) )  );
    m_inputSynchronizers.push_back( synchronizer );
    m_inputSynchronizers.back()->registerCallback(boost::bind(&PolygonOcclusionManager::newLaserscanAndSegmentationAvailable, this, _1,_2,m_numberLaserScanners));

    // Allocate buffer space for new laser
    LaserAndSegmentationBufferType dataBuffer(circular_buffer_size);
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

void PolygonOcclusionManager::newLaserscanAndSegmentationAvailable(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation, unsigned int laserID)
{
    ROS_DEBUG_STREAM("Received new laser scan and segmentation with id " << laserID << " and timestamp " << laserscan->header.stamp.toSec());
    
    // If it is first scan than initialize laser info struct
    if(m_laserInfos.at(laserID).numberMeasurements == 0)
    {

        m_laserInfos.at(laserID).frame_id = laserscan->header.frame_id;
        m_laserInfos.at(laserID).minAngle = angles::normalize_angle(laserscan->angle_min );
        m_laserInfos.at(laserID).maxAngle = angles::normalize_angle(laserscan->angle_max );
        m_laserInfos.at(laserID).angleIncrement = laserscan->angle_increment;
        m_laserInfos.at(laserID).numberMeasurements = (unsigned int) ((laserscan->angle_max -laserscan->angle_min)/laserscan->angle_increment)+1;
        ROS_WARN_STREAM("Added laser information for link " << m_laserInfos.at(laserID).frame_id << " min angle " << m_laserInfos.at(laserID).minAngle << " and max angle " << m_laserInfos.at(laserID).maxAngle);

    }

    // Get buffer for the laser depending on passed laserID
    LaserAndSegmentationBufferType& buffer = m_bufferVector.at(laserID);

    // save data in buffer
    LaserScanAndSegmentation newData = {laserscan,
                                        segmentation,
                                        Eigen::Affine3d::Identity(),
                                        laserID};
    LaserScanAndSegmentation::Ptr newDataPtr = boost::make_shared<LaserScanAndSegmentation>(newData);
    buffer.push_back(newDataPtr);
}

Tracks PolygonOcclusionManager::manageOcclusionsBeforeDataAssociation(Tracks& tracks, const ros::Time& time, const std::string& trackFrameID)
{
    if(!m_callbackQueue.isEmpty()){
        m_callbackQueue.callAvailable();
    }

    m_currenTrackerTime = time;
    initializeVisualization();
    m_occlusionRegionsPerLaser.clear();

    Tracks detectableTracks, occludedTracks;
    LaserAndSegmentationList laserStructPerLaser;
    for (unsigned int laserID = 0; laserID < m_bufferVector.size(); laserID++)
    {
        LaserAndSegmentationBufferType& buffer = m_bufferVector.at(laserID);
        LaserScanAndSegmentation::Ptr laserStruct = findCorrespondingDataInBuffer(time, buffer);

        if(!laserStruct)
        {

            ROS_WARN_STREAM("Occlusion Manager could not find suitable data in buffer for timestamp " << time);
            continue;
        }
        else
        {
            if(lookupTransformIntoSensorFrame(time,trackFrameID,laserStruct->laserData->header.frame_id, laserStruct->transformation))
            {
                ROS_DEBUG_STREAM("Found transormation for laser " << laserID);
                laserStructPerLaser.push_back(laserStruct);
                // Get polygons from laser data
                extractOcclusionRegions(laserStruct);
                visualizeOcclusionPolygons(laserStruct);
            }
            else
            {
                ROS_WARN_STREAM("Could not find transformation for laser " << laserID);
            }
        }
    }

    // Make sure tracks are in clean state
    foreach(Track::Ptr track, tracks)
    {
        //Set track state to MISSED for all tracks
        track->trackStatus = Track::MISSED;
        track->observation.reset();
    }

    const int NOT_IN_VIEW = -1;

    // Loop over set of tracks
    foreach(Track::Ptr track, tracks)
    {
        if(track->numberOfTotalMatches < m_minNumberMatches)
        {
            detectableTracks.push_back(track);
            continue;
        }

        Eigen::Vector3d transformedMean;
        Eigen::Matrix3d transformedCov;
        int trackIsObservableAtLaserID = NOT_IN_VIEW;
        int trackIsObservableAtBufferIdx = NOT_IN_VIEW;

        for(size_t i = 0; i < laserStructPerLaser.size(); i++){
            LaserScanAndSegmentation::Ptr& laserData = laserStructPerLaser.at(i);
            Eigen::Vector3d mean;
            Eigen::Matrix3d cov;
            transformTrackToSensorFrame(track,laserData->transformation,mean, cov);
            if(checkTrackForObservability(mean, laserData->laserID))
            {
                transformedMean = mean;
                transformedCov = cov;
                trackIsObservableAtLaserID = laserData->laserID;
                trackIsObservableAtBufferIdx =i;
            }
        }
        ROS_INFO_STREAM("Starting occlusion analysis for track " << track->id);
        if(m_occludedTracks.find(track->id) != m_occludedTracks.end())
        {
            ROS_DEBUG_STREAM("Track is contained in occluded track map.");
            OccludedTrack::Ptr occTrack = m_occludedTracks.at(track->id);
            if(trackIsObservableAtLaserID > NOT_IN_VIEW)
            {
                occTrack->laserID = trackIsObservableAtLaserID;
                track->trackStatus = Track::OCCLUDED;
                if(findOccludedTracks(occTrack, m_occlusionRegionsPerLaser.at(trackIsObservableAtBufferIdx), transformedMean, time, laserStructPerLaser.at(trackIsObservableAtBufferIdx)->transformation))
                {
                    occTrack->occlusionEndTime = time + ros::Duration(occTrack->distanceToEndOcclusion/occTrack->absoluteVelocity);
                    resetHigherOrderStateComponents(occTrack);
                    occludedTracks.push_back(track);
                }
                else if(findLikelyOccludedTracks(occTrack,m_occlusionRegionsPerLaser.at(trackIsObservableAtBufferIdx), transformedMean, time, laserStructPerLaser.at(trackIsObservableAtBufferIdx)->transformation))
                {
                    occTrack->occlusionEndTime = time + ros::Duration(occTrack->distanceToEndOcclusion/occTrack->absoluteVelocity);
                    resetHigherOrderStateComponents(occTrack);
                    occludedTracks.push_back(track);
                }
                else
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " is still contained in occlusion map but not occluded anymore at "
                                     << hypot((double)track->state->x()(STATE_X_IDX),(double)track->state->x()(STATE_Y_IDX)) << " or the current distance "
                                     << hypot((double)transformedMean(0),(double)transformedMean(1)) << ".");
                    track->detectionProbability = m_detectionProbabilityVisible;
                    zeroHigherOrderStateComponents(occTrack);
                    occludedTracks.push_back(track);
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Track " << track->id << " is not visible in any laser anymore but was in occluded map.");
                track->trackStatus = Track::MISSED;
                track->detectionProbability = m_detectionProbabilityOccluded;
                occludedTracks.push_back(track);
            }
        }
        else if(m_likelyOccludedTracks.find(track->id) != m_likelyOccludedTracks.end())
        {
            OccludedTrack::Ptr occTrack = m_likelyOccludedTracks.at(track->id);
            ROS_DEBUG_STREAM("Track is contained in likely occluded track map.");
            if(trackIsObservableAtLaserID > NOT_IN_VIEW)
            {
                occTrack->laserID = trackIsObservableAtLaserID;
                if(findOccludedTracks(occTrack, m_occlusionRegionsPerLaser.at(trackIsObservableAtBufferIdx), transformedMean, time, laserStructPerLaser.at(trackIsObservableAtBufferIdx)->transformation))
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was likely occluded and is now occluded");
                    // TODO do we just want matched updates in this case
                    //occTrack->stateBeginOcclusion = occTrack->track->state->deepCopy();
                    occTrack->occlusionEndTime = time + ros::Duration(occTrack->distanceToEndOcclusion/occTrack->absoluteVelocity);
                    track->numberOfConsecutiveMisses = 0;
                    m_occludedTracks.insert(std::pair<track_id, OccludedTrack::Ptr> (track->id,occTrack));
                    m_likelyOccludedTracks.erase(track->id);
                    occludedTracks.push_back(track);
                }
                else if(findLikelyOccludedTracks(occTrack, m_occlusionRegionsPerLaser.at(trackIsObservableAtBufferIdx), transformedMean, time, laserStructPerLaser.at(trackIsObservableAtBufferIdx)->transformation))
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was in likely occlusion map and stays likely occluded!");
                    occTrack->occlusionBeginTime = time + ros::Duration(occTrack->distanceToBeginOcclusion/occTrack->absoluteVelocity);
                    occTrack->occlusionEndTime = time + ros::Duration(occTrack->distanceToEndOcclusion/occTrack->absoluteVelocity);
                    detectableTracks.push_back(track);
                }
                else
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was in likely occlusion map but is neither occluded nor likely occluded!");
                    detectableTracks.push_back(track);
                    track->detectionProbability = m_detectionProbabilityVisible;
                    m_likelyOccludedTracks.erase(track->id);
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Track " << track->id << " is not visible in any laser anymore but was in likely occluded map");
                detectableTracks.push_back(track);
                track->detectionProbability = m_detectionProbabilityOccluded;
                track->trackStatus = Track::MISSED;
            }
        }
        else
        {
            if(trackIsObservableAtLaserID > NOT_IN_VIEW)
            {
                OccludedTrack::Ptr occTrack (new OccludedTrack);
                occTrack->track = track;
                occTrack->laserID = trackIsObservableAtLaserID;
                ROS_DEBUG_STREAM("Track is not yet contained in any occlusion map.");

                if(findOccludedTracks(occTrack, m_occlusionRegionsPerLaser.at(trackIsObservableAtBufferIdx), transformedMean, time, laserStructPerLaser.at(trackIsObservableAtBufferIdx)->transformation))
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " is not in any occlusion map, but occluded!");
                    occTrack->stateBeginOcclusion = occTrack->track->state->deepCopy();
                    occTrack->occlusionBeginTime = time;
                    occTrack->occlusionEndTime = time + ros::Duration(occTrack->distanceToEndOcclusion/occTrack->absoluteVelocity);
                    track->numberOfConsecutiveMisses = 0;
                    m_occludedTracks.insert(std::pair<track_id, OccludedTrack::Ptr> (track->id,occTrack));
                    occludedTracks.push_back(track);
                }
                else if(findLikelyOccludedTracks(occTrack, m_occlusionRegionsPerLaser.at(trackIsObservableAtBufferIdx), transformedMean, time, laserStructPerLaser.at(trackIsObservableAtBufferIdx)->transformation))
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " is not in any occlusion map, but likely occluded!");
                    occTrack->stateBeginOcclusion = occTrack->track->state->deepCopy();
                    occTrack->occlusionBeginTime = time + ros::Duration(occTrack->distanceToBeginOcclusion/occTrack->absoluteVelocity);
                    occTrack->occlusionEndTime = time + ros::Duration(occTrack->distanceToEndOcclusion/occTrack->absoluteVelocity);
                    m_likelyOccludedTracks.insert(std::pair<track_id, OccludedTrack::Ptr> (track->id,occTrack));
                    detectableTracks.push_back(track);
                }
                else
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " is not in any occlusion map, therefore detectable!");
                    track->detectionProbability = m_detectionProbabilityVisible;
                    detectableTracks.push_back(track);
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Track " << track->id << " is not visible in any laser anymore");
                track->detectionProbability = m_detectionProbabilityOccluded;
                detectableTracks.push_back(track);
                track->trackStatus = Track::MISSED;
            }
        }
    }
    visualizeOccludedTracks();
    tracks = detectableTracks;
    return occludedTracks;
}

void PolygonOcclusionManager::zeroHigherOrderStateComponents(OccludedTrack::Ptr occTrack)
{
    if(occTrack->numberConsecutiveNonOcclusions == m_numberOfReappereadFramesToZeroVelocity){
        StateVector zero = StateVector::Zero();
        zero.head(2) = occTrack->track->state->xp().head(2);
        occTrack->track->state->setXp(zero);
        ROS_DEBUG_STREAM("Setting higher order state terms to 0 for track " << occTrack->track->id
                        << "  state is now: \n" << occTrack->track->state->xp());
    }
    occTrack->numberConsecutiveNonOcclusions++;
}

void PolygonOcclusionManager::resetHigherOrderStateComponents(OccludedTrack::Ptr occTrack)
{
    if(occTrack->numberConsecutiveNonOcclusions >= m_numberOfReappereadFramesToZeroVelocity){
        StateVector reset = occTrack->stateBeginOcclusion->x();
        reset.head(2) = occTrack->track->state->xp().head(2);
        occTrack->track->state->setXp(reset);
        occTrack->numberConsecutiveNonOcclusions = 0;
        ROS_DEBUG_STREAM("Resetting higher order state terms to begin state for track " << occTrack->track->id
                        << "  state is now: \n" << occTrack->track->state->xp());
    }
}

bool PolygonOcclusionManager::checkTrackForObservability(const Eigen::Vector3d& trackMean, unsigned int laserID)
{
    ROS_INFO_STREAM("Check observability for laser " << laserID << " and mean " << trackMean(0) << "," <<trackMean(1));

    double phi = atan2((double)trackMean(1),(double)trackMean(0));
    double rho = trackMean.norm();

    ROS_INFO_STREAM("Angle bound check in frame_id "<< m_laserInfos.at(laserID).frame_id << " current phi is " << phi << " and threshold min " << m_laserInfos.at(laserID).minAngle << " max " << m_laserInfos.at(laserID).maxAngle);

    if(rho > m_occlusionMaxRange){
        ROS_INFO_STREAM("Track is not observable current distance is " << rho << " and threshold " << m_occlusionMaxRange);
        return false;
    }
    if(m_laserInfos.at(laserID).minAngle < m_laserInfos.at(laserID).maxAngle){
        if(phi < m_laserInfos.at(laserID).minAngle || phi > m_laserInfos.at(laserID).maxAngle){
            ROS_INFO_STREAM("Track is not observable in frame_id "<< m_laserInfos.at(laserID).frame_id << " current phi is " << phi << " and threshold min " << m_laserInfos.at(laserID).minAngle << " max " << m_laserInfos.at(laserID).maxAngle);
            return false;
        }
        else{
            if(phi > m_laserInfos.at(laserID).maxAngle && phi < m_laserInfos.at(laserID).minAngle){
                ROS_INFO_STREAM("Track is not observable in frame_id "<< m_laserInfos.at(laserID).frame_id << " current phi is " << phi << " and threshold min " << m_laserInfos.at(laserID).minAngle << " max " << m_laserInfos.at(laserID).maxAngle);
                return false;
            }
        }
    }
    return true;
}

PolygonOcclusionManager::LaserScanAndSegmentation::Ptr PolygonOcclusionManager::findCorrespondingDataInBuffer(const ros::Time& tracksTime, LaserAndSegmentationBufferType& buffer)
{
    LaserScanAndSegmentation::Ptr bestPtr;
    double smallestDiff = m_maximumSyncSlop;
    LaserAndSegmentationBufferType::size_type bestIdx = 0;
    for (LaserAndSegmentationBufferType::size_type i = 0; i < buffer.size(); i++)
    {
        ros::Duration duration = tracksTime - buffer.at(i)->laserData->header.stamp;
        if(fabs(duration.toSec()) < smallestDiff)
        {
            bestIdx = i;
            bestPtr = buffer.at(i);
            smallestDiff = fabs(duration.toSec());
            if(smallestDiff == 0)
                break;
        }

    }
    if(!bestPtr)
        buffer.rotate(buffer.begin() + bestIdx);

    ROS_DEBUG_STREAM("Size of buffer " << buffer.size() << " and time diff is " << smallestDiff << " for track time " << tracksTime.toSec());
    return bestPtr;
}

void PolygonOcclusionManager::extractOcclusionRegions(const LaserScanAndSegmentation::Ptr data)
{
    ROS_DEBUG("Extracting segments from scan");
    const sensor_msgs::LaserScan::ConstPtr& laserscan = data->laserData;
    const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation = data->segmentation;

    ROS_DEBUG_STREAM("Extracting polygons for laser with frame_id " << laserscan->header.frame_id << " containing "
                     << laserscan->ranges.size() << " measurements and " << segmentation->segments.size() <<  " segments.");
    OcclusionRegions regions;

    for (size_t i = 0; i < segmentation->segments.size(); i++)
    {
        srl_laser_segmentation::LaserscanSegment segment = segmentation->segments.at(i);
        // Assume measurement indices are ordered

        //Extracting segment data
        if(segment.measurement_indices.size() > 1)
        {
            SegmentInfo::Ptr info(new SegmentInfo);
            info->minAngle = angles::normalize_angle(laserscan->angle_min + laserscan->angle_increment * segment.measurement_indices.front()) ;
            info->maxAngle = angles::normalize_angle(laserscan->angle_min + laserscan->angle_increment * segment.measurement_indices.back()) ;
            info->distAtMinAngle = laserscan->ranges.at(segment.measurement_indices.front());
            info->distAtMaxAngle = laserscan->ranges.at(segment.measurement_indices.back());
            info->label = segment.label;
            info->idxSegmentation = i;

            if(info->distAtMinAngle < (m_occlusionMaxRange- m_selfOcclusionDistance) &&  info->distAtMaxAngle < (m_occlusionMaxRange- m_selfOcclusionDistance))
            {
                OcclusionRegion::Ptr occlusionRegion(new OcclusionRegion);
                occlusionRegion->responsableSegments.push_back(info);
                
                //Creating occlusion polygon
                Point2D pMinShade ((info->distAtMinAngle + m_selfOcclusionDistance) * cos(info->minAngle), (info->distAtMinAngle + m_selfOcclusionDistance) * sin(info->minAngle));
                occlusionRegion->occlusionPolygon.add_point(pMinShade);
                Point2D pMinShadeExtended(m_occlusionMaxRange * cos(info->minAngle),m_occlusionMaxRange * sin(info->minAngle));
                occlusionRegion->occlusionPolygon.add_point(pMinShadeExtended);
                
                const double segmentWidthRadians = fabs(info->minAngle - info->maxAngle);
                if(segmentWidthRadians > M_PI_4)
                {
                    int nIntermediatePoints = segmentWidthRadians/M_PI_4;
                    double interval = segmentWidthRadians/(double)nIntermediatePoints;
                    for (int i = 0; i < nIntermediatePoints; i++)
                    {
                        Point2D pIntermediate(m_occlusionMaxRange * cos(info->minAngle + (i+1)*interval),m_occlusionMaxRange * sin(info->minAngle + (i+1)*interval));
                        occlusionRegion->occlusionPolygon.add_point(pIntermediate);
                    }
                }

                Point2D pMaxShadeExtended(m_occlusionMaxRange * cos(info->maxAngle),m_occlusionMaxRange * sin(info->maxAngle));
                Point2D pMaxShade ((info->distAtMaxAngle + m_selfOcclusionDistance) * cos(info->maxAngle), (info->distAtMaxAngle + m_selfOcclusionDistance) * sin(info->maxAngle));
                occlusionRegion->occlusionPolygon.add_point(pMaxShadeExtended);
                occlusionRegion->occlusionPolygon.add_point(pMaxShade);
                occlusionRegion->occlusionPolygon.add_point(pMinShade);
                ROS_DEBUG_STREAM("Adding polygon with points [" << pMinShade << pMinShadeExtended << pMaxShadeExtended << pMaxShade );
                
                //TODO Make sure to create polygons th right way so we can save that correct step
                boost::geometry::correct(occlusionRegion->occlusionPolygon);

                // get polygons neighboring polygons where an occlusion in the future is likely to happen
                Polygon2D minPolygon, maxPolygon;
                Point2D diff = pMinShade - pMaxShade;
                diff = diff / hypot(diff.x, diff.y);
                Point2D pMinNeighbor = pMinShade + (diff * m_neighborPolygonWidth);
                Point2D pMinNeighborExtended = pMinShadeExtended + (diff * m_neighborPolygonWidth);
                Point2D pMaxNeighbor = pMaxShade - (diff * m_neighborPolygonWidth);
                Point2D pMaxNeighborExtended = pMaxShadeExtended - (diff * m_neighborPolygonWidth);
                minPolygon.add_point(pMinShade);
                minPolygon.add_point(pMinNeighbor);
                minPolygon.add_point(pMinNeighborExtended);
                minPolygon.add_point(pMinShadeExtended);
                minPolygon.add_point(pMinShade);
                boost::geometry::correct(minPolygon);

                maxPolygon.add_point(pMaxShade);
                maxPolygon.add_point(pMaxNeighbor);
                maxPolygon.add_point(pMaxNeighborExtended);
                maxPolygon.add_point(pMaxShadeExtended);
                maxPolygon.add_point(pMaxShade);
                boost::geometry::correct(maxPolygon);

                occlusionRegion->occlusionRiskPolygons.push_back(minPolygon);
                occlusionRegion->occlusionRiskPolygons.push_back(maxPolygon);
                //TODO Thinka about an area Threshold
                //double area = boost::geometry::area(occlusionRegion->occlusionPolygon);

                //ROS_INFO_STREAM("Polygon point size " << occlusionRegion->occlusionPolygon.point_count() << "with area " << area);
                regions.push_back(occlusionRegion);
            }
        }
    }
    m_occlusionRegionsPerLaser.push_back(regions);
}

bool PolygonOcclusionManager::findOccludedTracks(OccludedTrack::Ptr occTrack, const OcclusionRegions& occlusionRegions, Eigen::Vector3d& meanTrack, const ros::Time& time, const Eigen::Affine3d& transform)
{
    bool occlusionFound = false;
    Point2D trackPoint(meanTrack(0), meanTrack(1));
    FilterState::Ptr currentState;
    if(occTrack->stateBeginOcclusion)
        currentState = occTrack->stateBeginOcclusion;
    else
        currentState = occTrack->track->state;

    //Transform velocity
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    velocity(0) = currentState->xp()(STATE_VX_IDX);
    velocity(1) = currentState->xp()(STATE_VY_IDX);
    Eigen::Vector3d transformedVelocity = transform.rotation() * velocity;

    ROS_DEBUG_STREAM("Current rotation matrix is \n" << transform.rotation());
    ROS_DEBUG_STREAM("TrackerFrame Velocities are (" << currentState->xp()(STATE_VX_IDX) << ";" << currentState->xp()(STATE_VY_IDX) <<
                     ") transformed to (" << transformedVelocity(0) << ";" << transformedVelocity(1) << ")");

    foreach(OcclusionRegion::Ptr region, occlusionRegions)
    {
        if(boost::geometry::within(trackPoint, region->occlusionPolygon))
        {
            const double extendDistance = 1e6;
            // Store the values for fast access and easy
            // equations-to-code conversion
            double absVelocity = hypot((double)transformedVelocity(0),(double)transformedVelocity(1));
            absVelocity = std::max(m_minAbsoluteVelocity, absVelocity);
            Linestring2D trackLine;
            Point2D point;
            point.x = meanTrack(0) + (transformedVelocity(0)/absVelocity * extendDistance);
            point.y = meanTrack(1) + (transformedVelocity(1)/absVelocity * extendDistance);
            trackLine.push_back(trackPoint);
            trackLine.push_back(point);

            occTrack->absoluteVelocity = absVelocity;

            std::vector<Point2D> intersections;
            boost::geometry::intersection(region->occlusionPolygon, trackLine, intersections);
            if(intersections.size() == 1)
            {
                occTrack->distanceToEndOcclusion = trackPoint.distance(intersections.at(0));
            }
            else
            {
                ROS_ERROR_STREAM("Track " << occTrack->track->id << " is in occlusion polygon and we found " << intersections.size() << " intersections! Something is definitely going wrong!\nTrack state is" << transformedVelocity);
                continue;
            }
            //set detection probability and mark track as occluded
            occTrack->track->trackStatus = Track::OCCLUDED;
            occTrack->track->detectionProbability = m_detectionProbabilityOccluded;
            visualizeOcclusionDistance(trackPoint, intersections.front(), occTrack->track->id, m_laserInfos.at(occTrack->laserID).frame_id);
            occlusionFound = true;
            break;
        }
    }
    return occlusionFound;
}

bool PolygonOcclusionManager::findLikelyOccludedTracks(OccludedTrack::Ptr occTrack, const OcclusionRegions& occlusionRegions, Eigen::Vector3d& meanTrack, const ros::Time& time, const Eigen::Affine3d& transform)
{
    bool likelyOccludedFound = false;
    Point2D trackPoint(meanTrack(0), meanTrack(1));
    FilterState::Ptr currentState;
    if(occTrack->stateBeginOcclusion)
        currentState = occTrack->stateBeginOcclusion;
    else
        currentState = occTrack->track->state;
    //Transform velocity
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    velocity(0) = currentState->xp()(STATE_VX_IDX);
    velocity(1) = currentState->xp()(STATE_VY_IDX);
    Eigen::Vector3d transformedVelocity = transform.rotation() * velocity;
    foreach(OcclusionRegion::Ptr region, occlusionRegions)
    {
        foreach(Polygon2D polygon, region->occlusionRiskPolygons)
        {
            if(boost::geometry::within(trackPoint, polygon))
            {
                const double extendDistance = 1e6;
                // Store the values for fast access and easy
                // equations-to-code conversion
                double absVelocity = hypot((double)transformedVelocity(0),(double)transformedVelocity(1));
                absVelocity = std::max(m_minAbsoluteVelocity, absVelocity);
                Linestring2D trackLine;
                Point2D point;
                point.x = meanTrack(0) + (transformedVelocity(0)/absVelocity * extendDistance);
                point.y = meanTrack(1) + (transformedVelocity(1)/absVelocity * extendDistance);
                trackLine.push_back(trackPoint);
                trackLine.push_back(point);

                occTrack->absoluteVelocity = absVelocity;
                std::vector<Point2D> intersections;
                boost::geometry::intersection(region->occlusionPolygon, trackLine, intersections);
                if(intersections.size() == 2)
                {
                    occTrack->distanceToBeginOcclusion = trackPoint.distance(intersections.at(0));
                    occTrack->distanceToEndOcclusion = trackPoint.distance(intersections.at(1));
                    if(occTrack->distanceToBeginOcclusion > occTrack->distanceToEndOcclusion)
                    {
                        double temp = occTrack->distanceToBeginOcclusion;
                        occTrack->distanceToBeginOcclusion = occTrack->distanceToEndOcclusion;
                        occTrack->distanceToEndOcclusion = temp;
                    }
                    double occProb = calculateOcclusionProbability(occTrack->distanceToBeginOcclusion);
                    setDetectionProbability(occProb, occTrack->track);
                    likelyOccludedFound = true;
                    return likelyOccludedFound;
                }
                else if(intersections.size() == 0)
                {
                    double distance = boost::geometry::distance(region->occlusionPolygon, trackPoint);
                    double occProb = calculateOcclusionProbability(distance);
                    setDetectionProbability(occProb, occTrack->track);
                    ROS_DEBUG_STREAM("No intersection with occlusion shade because person with id " << occTrack->track->id << " is walking parallel or away from shade. Distance to shade is "  << distance << " and occlusion Probability therefore " << occProb);
                }
            }
        }
    }
    return likelyOccludedFound;
}

bool PolygonOcclusionManager::lookupTransformIntoSensorFrame(ros::Time stamp, const std::string& sourceFrame, const std::string& targetFrame, Eigen::Affine3d& resultingTransform)
{
    ROS_DEBUG_STREAM("Entering transform lookup from " << sourceFrame << " to " << targetFrame << " for time " << stamp.toNSec());
    tf::StampedTransform tfTransform;

    try {
        m_transformListener.waitForTransform(targetFrame, sourceFrame, stamp, ros::Duration(srl_nnt::Params::get<double>("transform_timeout", 0.01) ));
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

void PolygonOcclusionManager::transformTrackToSensorFrame(Track::Ptr track, Eigen::Affine3d& transform, Eigen::Vector3d& meanTrackLF, Eigen::Matrix3d& covTrackLF)
{
    Eigen::Vector3d trackPosition = Eigen::Vector3d::Zero();
    trackPosition(0) = track->state->xp()(0);
    trackPosition(1) = track->state->xp()(1);

    // ROS Covariance is a 6x6 matrix (xyz + xyz rotation) relative to the message's coordinate frame
    // We are not interested in pose rotation, so only take first 3 rows and columns
    Eigen::Matrix3d trackCov = Eigen::Matrix3d::Identity();
    trackCov(0, 0) = track->state->Cp()(0,0);
    trackCov(1, 0) = track->state->Cp()(1,0);
    trackCov(0, 1) = track->state->Cp()(0,1);
    trackCov(1, 1) = track->state->Cp()(1,1);
    //set to one for cholesky solver to work

    // Transform pose and covariance into our reference frame, still in 3D
    meanTrackLF = transform * trackPosition;

    // For covariance, only the coordinate frame rotation is relevant (invariant w.r.t. translation)
    Eigen::Matrix3d detectionFrameToOurFrameRotation = transform.linear().matrix();
    covTrackLF = detectionFrameToOurFrameRotation * trackCov * detectionFrameToOurFrameRotation.transpose();
    /*ROS_INFO_STREAM("Mean transformed from \n" << trackPosition << " to sensor frame \n" << meanTrackLF);
        ROS_INFO_STREAM("Covariance transformed from \n" << trackCov << " to sensor frame \n" << covTrackLF);*/
}

double PolygonOcclusionManager::calculateOcclusionProbability(const double distanceToShade)
{
    //LINEAR COST FUNFCTION
    if(distanceToShade > m_neighborPolygonWidth)
    {
        return 0.0;
    }
    else
    {
        return (m_neighborPolygonWidth-distanceToShade)/m_neighborPolygonWidth;
    }
}

void PolygonOcclusionManager::setDetectionProbability(const double occlusionProbability, const Track::Ptr track)
{

    track->detectionProbability = m_detectionProbabilityOccluded + (m_detectionProbabilityVisible - m_detectionProbabilityOccluded) * (1-occlusionProbability);
    //  if(track->detectionProbability < 0.5)
    //    track->trackStatus = Track::OCCLUDED;
}

Pairings PolygonOcclusionManager::occludedTrackAssociation(Tracks tracks, Observations observations, const ros::Time& time)
{
    assert(tracks.size() == m_occludedTracks.size());

    // Get unmatched observations
    Observations unmatchedObservations;
    foreach(Observation::Ptr observation, observations)
    {
        if(!observation->matched)
            unmatchedObservations.push_back(observation);

    }
    const double MATRIX_LN_EPS = -1e8;

    Pairings compatiblePairings;
    Eigen::MatrixXd reassignmentCost = Eigen::MatrixXd::Constant(m_occludedTracks.size(), unmatchedObservations.size(), BIG_COST);

    /// Calculate occlusion cost
    bool foundAtLeastOnePossibleAssociation = false;
    for (size_t i_ob=0; i_ob < unmatchedObservations.size(); i_ob++)
    {
        Observation::Ptr observation = unmatchedObservations.at(i_ob);
        unsigned int i_track = 0;
        for (OccludedTrackMap::iterator it_track=m_occludedTracks.begin(); it_track != m_occludedTracks.end(); ++it_track)
        {
            double current_cost = calcAssociationCost(it_track->second, observation, time);
            reassignmentCost(i_track,i_ob) = current_cost;
            if(current_cost < BIG_COST)
                foundAtLeastOnePossibleAssociation = true;
            i_track++;
        }
    }

    if(foundAtLeastOnePossibleAssociation)
    {
        LAPSolver<double> linearAssignmentProblem;
        Eigen::VectorXi trackAssignments, obsAssignments;

        ROS_DEBUG_STREAM("Cost Matrix " << reassignmentCost);

        double totalCost = linearAssignmentProblem.calculateAssignment(reassignmentCost, trackAssignments, obsAssignments);

        unsigned int i_track = 0;
        for (OccludedTrackMap::iterator it_track=m_occludedTracks.begin(); it_track != m_occludedTracks.end(); ++it_track)
        {
            double currentCost = reassignmentCost(i_track, obsAssignments(i_track));
            if(currentCost < BIG_COST)
            {
                Pairing::Ptr pairing( new Pairing );
                Track::Ptr track = it_track->second->track;
                pairing->track = track;
                pairing->observation = unmatchedObservations.at(obsAssignments(i_track));
                pairing->validated = false;
                visualizePossibleOcclusionAssignments(pairing, currentCost);

                // Calculate innovation v and inverse of innovation covariance S
                pairing->v = pairing->observation->z - pairing->track->state->zp();

                // TODO wie stark das update uebernommen werden soll
                ObsMatrix S = ObsMatrix::Zero();
                if(!m_scaleUpdateWithCost)
                    S = pairing->track->state->H() * pairing->track->state->Cp() * pairing->track->state->H().transpose() + pairing->observation->R;
                else
                    S = pairing->track->state->H() * pairing->track->state->Cp() * pairing->track->state->H().transpose() + (pairing->observation->R*currentCost);
                Eigen::FullPivLU<ObsMatrix> lu(S);
                double ln_det_S = 0;
                ln_det_S = log(lu.determinant());

                // Calculate inverse of innovation covariance if possible
                if(ln_det_S > MATRIX_LN_EPS)
                {
                    pairing->Sinv = lu.inverse();
                    pairing->d = (pairing->v.transpose() * pairing->Sinv * pairing->v)(0,0);
                    pairing->singular = pairing->d < 0.0;
                }
                else
                {
                    pairing->Sinv = ObsMatrix::Constant(OBS_DIM, OBS_DIM, numeric_limits<double>::quiet_NaN());
                    pairing->d = numeric_limits<double>::quiet_NaN();
                    pairing->singular = true;

                    ROS_WARN_STREAM("Singular pairing encountered!\nTrack " << pairing->track->id << " measurement prediction:\n" << pairing->track->state->zp() << "\nTrack covariance prediction:\n" << pairing->track->state->Cp()
                                    << "\nObservation " << pairing->observation->id << " mean:\n" << pairing->observation->z << "\nObservation covariance:\n" << pairing->observation->R );
                }

                // Check for singularity
                if(!pairing->singular)
                {
                    // Store in list of compatible pairings
                    compatiblePairings.push_back(pairing);
                    ROS_INFO_STREAM("Current cost = " << currentCost);
                    ROS_INFO_STREAM("Occlusion Geodesics reassignment of track " << track->id << " to ob "<< pairing->observation->id <<  ": \n" <<
                                    "Current time:" << time.toSec() << "\tOcclusionStartTime:" << it_track->second->occlusionBeginTime.toSec() << "\tOcclusionEndTime:" << it_track->second->occlusionEndTime.toSec() << "\n" <<
                                    "Occlusion therefore took " << ros::Duration(time-it_track->second->occlusionBeginTime).toSec() << " expected were " << ros::Duration(it_track->second->occlusionEndTime-it_track->second->occlusionBeginTime).toSec()<< "\n" <<
                                    "Current track prediction: (" << track->state->xp()(0) << track->state->xp()(1) << ")\n" <<
                                    "Associated observation: (" << pairing->observation->z(0) << pairing->observation->z(1) << ")\n" <<
                                    "Gap begin to Observation: " << (it_track->second->stateBeginOcclusion->x().head(2)-pairing->observation->z).norm() << "\n" <<
                                    "Occlusion cost: " << currentCost);
                    // Track has been matched
                    compatiblePairings.back()->validated = true;
                    compatiblePairings.back()->observation->matched = true;
                    track->observation = compatiblePairings.back()->observation;
                    track->trackStatus = Track::MATCHED;
                    track->numberOfTotalMatches++;
                    track->numberOfConsecutiveOcclusions = 0;
                    track->numberOfConsecutiveMisses = 0;
                }
                //if pairing is singular
                else
                {
                    it_track->second->track->observation.reset();
                    switch (it_track->second->track->trackStatus){
                        case Track::MISSED:
                            it_track->second->track->numberOfConsecutiveMisses++;
                            break;
                        case Track::OCCLUDED:
                            it_track->second->track->numberOfConsecutiveOcclusions++;
                            break;
                    }
                }
            }
            //if no assignment could be found
            else  {
                it_track->second->track->observation.reset();
                switch (it_track->second->track->trackStatus){
                    case Track::MISSED:
                        it_track->second->track->numberOfConsecutiveMisses++;
                        break;
                    case Track::OCCLUDED:
                        it_track->second->track->numberOfConsecutiveOcclusions++;
                        break;
                }
            }
            i_track++;
        }
        if(compatiblePairings.size() > 0)
            ROS_WARN_STREAM("Reassigned " << compatiblePairings.size() << " occluded tracks.");
        foreach(Pairing::Ptr pairing, compatiblePairings){
            m_occludedTracks.erase(pairing->track->id);
        }
        visualizeOcclusionAssignments(compatiblePairings);
    }
    //if there was no track with possible assignment we keep all tracks
    else
    {
        for (OccludedTrackMap::iterator it_track=m_occludedTracks.begin(); it_track != m_occludedTracks.end(); ++it_track)
        {
            it_track->second->track->observation.reset();
            switch (it_track->second->track->trackStatus){
                case Track::MISSED:
                    it_track->second->track->numberOfConsecutiveMisses++;
                    break;
                case Track::OCCLUDED:
                    it_track->second->track->numberOfConsecutiveOcclusions++;
                    break;
            }
        }
    }
    return compatiblePairings;
}

double PolygonOcclusionManager::calcAssociationCost(OccludedTrack::Ptr occTrack, Observation::Ptr observation, const ros::Time& time)
{
    /// Calculate occlusion cost
    // According to the paper this detector reliability is comparable to the detection Probability
    double c_occlusion = occTrack->track->detectionProbability;
    if(!m_occlusion_geodesics_use_detection_probability)
        c_occlusion = 1.0;

    // Calculate plausibility term
    const double motion_variance = m_occlusion_geodesics_motion_variance;
    const double average_velocity = 1.5;
    Eigen::Vector2d d_j = occTrack->stateBeginOcclusion->x().head(2) - observation->z.head(2);
    double numerator = d_j.squaredNorm();
    Eigen::Vector2d d_i = occTrack->stateBeginOcclusion->x().head(2)- occTrack->track->state->xp().head(2);
    double d_i_norm= d_i.norm();
    double d_i_avg = ros::Duration(time - occTrack->occlusionBeginTime).toSec() * average_velocity;
    const double average_distance_per_frame = 0.15;
    double d_cov = 3* sqrt(occTrack->stateBeginOcclusion->Cp()(0,0) + occTrack->stateBeginOcclusion->Cp()(1,1));
    double d_i_compare = max(d_i_avg, d_cov);
    double d_i_squared = max(d_i_norm,d_i_compare) * max(d_i_norm,d_i_compare);
    double denominator = 2 * motion_variance * d_i_squared;
    double c_plausible = 1.0;

    if(m_occlusion_geodesics_use_plausibility) {
        c_plausible = std::exp(-(numerator/denominator));
        if(c_plausible < m_occlusion_geodesics_plausibility_cut_off) {
            ROS_INFO_STREAM("Occlusion plausible cost for track " << occTrack->track->id << " and observation " << observation->id << " at "
                            << observation->z(0) << ";" << observation->z(1) << " exceeds threshold " << c_plausible
                            << " dj (begin->obs) " <<  d_j.norm() << " d_i(begin->predict) " << d_i.norm()
                            << " d_i(begin->v_avg) " << d_i_compare << " d_cov " << d_cov << " distance in shade " << occTrack->distanceToEndOcclusion);
            c_plausible = std::numeric_limits<double>::infinity();
        }
        else
        {
            ROS_INFO_STREAM("Occlusion plausible cost for track " << occTrack->track->id << " and observation " << observation->id << " at "
                            << observation->z(0) << ";" << observation->z(1) << " is  " << c_plausible
                            << "\ndj (begin->obs) " <<  d_j.norm() << " d_i(begin->predict) " << d_i.norm()
                            << " d_i(begin->v_avg) " << d_i_compare << " d_cov " << d_cov << " distance in shade " << occTrack->distanceToEndOcclusion);
        }
    }

    // Inverted Plausibility cost
    Eigen::Vector2d d_k = occTrack->track->state->xp().head(2) - observation->z.head(2);
    numerator = d_k.squaredNorm();
    denominator = 2 * motion_variance * d_i_squared;
    double c_plausible_inv = std::exp(-(numerator/denominator));
    if(!m_occlusion_geodesics_use_inverted_plausibility)
        c_plausible_inv = 1.0;

    // Calculate Inertia term
    double dot_result = d_i.dot(d_j);
    double  inertia_variance = m_occlusion_geodesics_inertia_variance;
    if(d_i_norm < 0.3)
        inertia_variance = 2* M_PI;
    double di_dj_norm_product = d_i.norm() * d_j.norm();
    numerator = (dot_result - di_dj_norm_product) * (dot_result - di_dj_norm_product);
    denominator = 2* inertia_variance * d_i.squaredNorm() * d_j.squaredNorm();
    double c_inertia = std::exp(- numerator/denominator);

    // Calculate Duration cost
    double c_duration = 1.0;
    if(m_occlusion_geodesics_use_duration_cost)
    {
        ros::Duration diff(m_currenTrackerTime - occTrack->occlusionEndTime);
        if(diff.toSec() > 0)
            c_duration = std::exp(- diff.toSec()/m_allowedDurationForReappearance);
    }

    double cost_product = (c_occlusion * c_plausible * c_inertia * c_plausible_inv * c_duration);
    if(cost_product < m_occlusion_geodesics_total_cut_off)
    {
        ROS_INFO_STREAM("Cost product very low " << cost_product);
        cost_product = std::numeric_limits<double>::infinity();
    }
    double total_cost =1 - cost_product;

    if(std::isnan(total_cost) || std::isinf(total_cost))
        total_cost = BIG_COST;

    ROS_INFO_STREAM("Occlusion reassignment cost for track " << occTrack->track->id << " and observation " << observation->id << " at "
                    << observation->z(0) << ";" << observation->z(1) << " is " << total_cost << " \n(Occlusion: " << c_occlusion
                    << "; Plausible: " << c_plausible << "; PlausibleInv: " << c_plausible_inv <<"; Inertia: "
                    << c_inertia << "; Duration: "<< c_duration <<  ")");
    return total_cost;
}


/// *****************************************************************
/// VISUALIZATION STUFF
/// *****************************************************************

void PolygonOcclusionManager::initializeVisualization()
{
    if(m_visualizationEnabled)
    {
        foreach(visualization_msgs::Marker& marker, m_currentMarkers.markers) {
            marker.action = visualization_msgs::Marker::DELETE;
        }
        m_visualizationPublisher.publish(m_currentMarkers);
        m_currentMarkers.markers.clear();
    }
}

void PolygonOcclusionManager::publishMarkerArray()
{
    if(m_visualizationEnabled)
    {
        m_visualizationPublisher.publish(m_currentMarkers);
    }
}

void PolygonOcclusionManager::visualizeOcclusionDistance(Point2D& trackPosition, Point2D& intersection, const unsigned int trackID, const std::string& frame_id)
{
    if(m_visualizationEnabled)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = m_currenTrackerTime;

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
        point.x = trackPosition.x;
        point.y = trackPosition.y;

        marker.points.push_back(point);
        point.x = intersection.x;
        point.y = intersection.y;

        marker.points.push_back(point);

        m_currentMarkers.markers.push_back(marker);
    }

}

void PolygonOcclusionManager::visualizeOcclusionPolygons(LaserScanAndSegmentation::Ptr laserData)
{
    if(m_visualizationEnabled)
    {
        visualization_msgs::Marker marker, markerRisk;
        marker.header.stamp = laserData->laserData->header.stamp;
        marker.id = 0;

        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.lifetime = ros::Duration();
        marker.scale.x = 0.05;

        marker.ns = "occlusion_polygons";

        markerRisk.header.stamp = laserData->laserData->header.stamp;
        markerRisk.id = 0;
        markerRisk.action = visualization_msgs::Marker::ADD;
        markerRisk.type = visualization_msgs::Marker::LINE_STRIP;
        markerRisk.lifetime = ros::Duration();
        markerRisk.scale.x = 0.05;
        markerRisk.ns = "occlusion_risk_polygons";


        for(size_t i = 0; i< m_occlusionRegionsPerLaser.size(); i++){
            marker.header.frame_id = markerRisk.header.frame_id = m_laserInfos.at(i).frame_id;
            OcclusionRegions& regions = m_occlusionRegionsPerLaser.at(i);
            markerRisk.color.g = (rand()/(double)RAND_MAX);
            markerRisk.color.a = 1;
            markerRisk.color.r = (rand()/(double)RAND_MAX);
            markerRisk.color.b = (rand()/(double)RAND_MAX);
            marker.color.g = (rand()/(double)RAND_MAX);
            marker.color.b = (rand()/(double)RAND_MAX);
            marker.color.g = (rand()/(double)RAND_MAX);
            marker.color.a = 1;

            foreach(OcclusionRegion::Ptr region, regions){
                region->occlusionPolygon.get_points_for_marker(marker);
                m_currentMarkers.markers.push_back(marker);
                marker.id++;
                foreach(Polygon2D polygon, region->occlusionRiskPolygons){
                    polygon.get_points_for_marker(markerRisk);
                    m_currentMarkers.markers.push_back(markerRisk);
                    markerRisk.id++;
                }
            }
        }
    }
}

void PolygonOcclusionManager::visualizeOccludedTracks()
{
    if(m_visualizationEnabled && m_occludedTracks.size() > 0)
    {
        visualization_msgs::Marker markerOccludedTracks, markerPrediction;

        // Common settings
        markerOccludedTracks.header.frame_id = markerPrediction.header.frame_id = m_frameIDTracker;
        markerOccludedTracks.header.stamp = markerPrediction.header.stamp = m_currenTrackerTime;
        markerOccludedTracks.action = markerPrediction.action =visualization_msgs::Marker::ADD;
        markerOccludedTracks.ns = markerPrediction.ns ="occlusion_begin_state";

        // Marker for begin off occlusion cube
        markerOccludedTracks.id = 1 ;
        markerOccludedTracks.type =  visualization_msgs::Marker::CUBE_LIST;
        markerOccludedTracks.color.g = 1.0f;
        markerOccludedTracks.color.a = 1.0;
        markerOccludedTracks.scale.x = 0.2;
        markerOccludedTracks.scale.y = 0.2;
        markerOccludedTracks.scale.z = 0.2;

        // Marker for showing connection between current prediction and begin of occlusion
        markerPrediction.type =  visualization_msgs::Marker::LINE_LIST;
        markerPrediction.id =2 ;
        markerPrediction.color.g = 1.0f;
        markerPrediction.color.a = 1.0;
        markerPrediction.scale.x = 0.05;

        for (OccludedTrackMap::iterator it_track=m_occludedTracks.begin(); it_track != m_occludedTracks.end(); ++it_track)
        {
            OccludedTrack::Ptr& occTrack = it_track->second;

            geometry_msgs::Point p;
            p.x = occTrack->stateBeginOcclusion->x()(0);
            p.y = occTrack->stateBeginOcclusion->x()(1);
            p.z = 0.2;

            markerOccludedTracks.points.push_back(p);
            markerPrediction.points.push_back(p);

            p.x = occTrack->track->state->x()(0);
            p.y = occTrack->track->state->x()(1);
            p.z = 0.2;
            markerPrediction.points.push_back(p);
        }
        m_currentMarkers.markers.push_back(markerOccludedTracks);
        m_currentMarkers.markers.push_back(markerPrediction);
    }
}

void PolygonOcclusionManager::visualizePossibleOcclusionAssignments(Pairing::Ptr pairing, const double cost)
{
    if(m_visualizationEnabled && pairing)
    {
        visualization_msgs::Marker markerAssignment;
        markerAssignment.header.frame_id = m_frameIDTracker;
        markerAssignment.header.stamp = m_currenTrackerTime;

        markerAssignment.id = pairing->track->id + pairing->observation->id ;

        markerAssignment.action =visualization_msgs::Marker::ADD;
        markerAssignment.type =  visualization_msgs::Marker::ARROW;

        markerAssignment.ns = "occlusion_possible_assignments";

        markerAssignment.scale.x = 0.08;
        markerAssignment.scale.y = 0.2;
        markerAssignment.scale.z = 0.3;


        markerAssignment.color.g = JetColors::green(1-cost);
        markerAssignment.color.b = JetColors::blue(1-cost);
        markerAssignment.color.r = JetColors::red(1-cost);
        markerAssignment.color.a = 0.8;

        markerAssignment.lifetime = ros::Duration();

        geometry_msgs::Point point;
        point.x = pairing->track->state->x()(0);
        point.y = pairing->track->state->x()(1);

        markerAssignment.points.push_back(point);
        point.x = pairing->observation->z(0);
        point.y = pairing->observation->z(1);
        markerAssignment.points.push_back(point);
        m_currentMarkers.markers.push_back(markerAssignment);
    }
}

void PolygonOcclusionManager::visualizeOcclusionAssignments(Pairings pairings)
{
    if(m_visualizationEnabled)
    {
        if(pairings.size() > 0)
        {
            visualization_msgs::Marker markerConnection;
            markerConnection.header.frame_id = m_frameIDTracker;
            markerConnection.header.stamp = m_currenTrackerTime;
            unsigned int markerID = 0;
            markerConnection.action =visualization_msgs::Marker::ADD;
            markerConnection.type =  visualization_msgs::Marker::ARROW;

            markerConnection.ns = "occlusion_assignments";

            markerConnection.scale.x = 0.05;
            markerConnection.scale.y = 0.2;
            markerConnection.scale.z = 0.3;
            markerConnection.lifetime = ros::Duration();

            foreach(Pairing::Ptr pairing, pairings)
            {
                markerConnection.id = markerID;

                // Line strip is blue
                markerConnection.color.g = 0.58;
                markerConnection.color.b = 0.604;
                markerConnection.color.a = 0.8;
                geometry_msgs::Point point;

                // Start is current state prediction
                point.x = pairing->track->state->x()(0);
                point.y = pairing->track->state->x()(1);
                markerConnection.points.push_back(point);

                // End is observation where occluded track is assigned to
                point.x = pairing->observation->z(0);
                point.y = pairing->observation->z(1);
                markerConnection.points.push_back(point);
                m_currentMarkers.markers.push_back(markerConnection);
                
                markerID++;
            }
        }
    }
}



}

