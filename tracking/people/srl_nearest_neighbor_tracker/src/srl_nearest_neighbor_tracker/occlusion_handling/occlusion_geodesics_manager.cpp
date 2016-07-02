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
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_geodesics_manager.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/base/lap.h>

#define foreach BOOST_FOREACH


namespace srl_nnt {

OcclusionGeodesicsManager::OcclusionGeodesicsManager() :
  m_numberLaserScanners(0), m_visualizationEnabled(false), m_detectionProbabilityOccluded(0.2), m_detectionProbabilityVisible(0.9),
  m_numCyclesTotal(0), m_numCyclesWithMissingSensorData(0)
{
}


void OcclusionGeodesicsManager::initializeOcclusionManager(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle)
{
    // Save nodehandles in members
    m_nodeHandle = nodeHandle;
    m_privateNodeHandle = privateNodeHandle;

    // Initialize sampler
    m_visualizationEnabled = Params::get<bool>("occlusion_manager_visualization_enabled", false);
    m_occlusionMaxRange = Params::get<double>("occlusion_manager_max_range",60);
    m_neighborPolygonWidth = Params::get<double>("occlusion_neigbor_polygon_width",0.2);
    m_selfOcclusionDistance = Params::get<double>("occlusion_self_occlusion_distance",0.25);
    m_allowedDurationForReappearance = Params::get<double>("occlusion_allowed_duration_for_reappearance",2.0);
    m_scaleUpdateWithCost = Params::get<bool>("occlusion_geodesics_scale_reassignment",false);
    m_maximumSyncSlop = Params::get<double>("occlusion_manager_maximum_sync_slop",0.04);
    m_minNumberMatches = Params::get<int>("occlusion_manager_minimum_matches",0);
    m_minAbsoluteVelocity = Params::get<int>("occlusion_manager_minimum_absolute_velocity",0.1);

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
    m_occlusion_geodesics_grid_dimension_forward_meter = Params::get<double>("occlusion_geodesics_grid_dimension_forward_meter", 4.0);
    m_occlusion_geodesics_grid_dimension_backward_meter = Params::get<double>("occlusion_geodesics_grid_dimension_backward_meter", 1.0);
    m_occlusion_geodesics_grid_dimension_width_meter = Params::get<double>("occlusion_geodesics_grid_dimension_width_meter", 4.0);

    m_occlusion_geodesics_grid_cell_resolution_meter = Params::get<double>("occlusion_geodesics_grid_cell_resolution_meter", 0.1);
    m_occlusion_geodesics_infimum_radius = Params::get<int>("occlusion_geodesics_infimum_radius", 2);
    m_detectionProbabilityVisible = Params::get<double>("occlusion_geodesics_detector_reliability", 0.8);
    m_occlusion_geodesics_number_of_assignments_before_acceptance = Params::get<int>("occlusion_geodesics_number_of_assignments_before_acceptance",1);
    m_occlusion_geodesics_allowed_orientation_difference_for_acceptance = Params::get<int>("occlusion_geodesics_allowed_orientation_difference",M_PI);

    ROS_INFO_STREAM("#### Polygon Occlusion Manager configured as follows: #####\n "
            << "occlusion_manager_visualization_enabled:" << m_visualizationEnabled << "\n"
            << "occlusion_manager_max_range:" << m_occlusionMaxRange << "\n"
            << "occlusion_manager_number_lasers:" << laserSubscriptions << "\n"
            << "occlusion_geodesics_inertia_variance:" << m_occlusion_geodesics_inertia_variance << "\n"
            << "occlusion_geodesics_motion_variance:" << m_occlusion_geodesics_motion_variance << "\n"
            << "occlusion_geodesics_plausibility_cut_off:" << m_occlusion_geodesics_plausibility_cut_off << "\n"
            << "occlusion_geodesics_grid_dimension_forward_meter:" << m_occlusion_geodesics_grid_dimension_forward_meter << "\n"
            << "occlusion_geodesics_grid_dimension_backward_meter:" << m_occlusion_geodesics_grid_dimension_backward_meter << "\n"
            << "occlusion_geodesics_grid_dimension_width_meter:" << m_occlusion_geodesics_grid_dimension_width_meter << "\n"

            << "occlusion_geodesics_grid_cell_resolution_meter:" << m_occlusion_geodesics_grid_cell_resolution_meter << "\n"
            << "occlusion_geodesics_infimum_radius:" << m_occlusion_geodesics_infimum_radius << "\n"
            << "occlusion_geodesics_detector_reliability:" << m_detectionProbabilityVisible << "\n"
            << "occlusion_geodesics_number_of_assignments_before_acceptance:" << m_occlusion_geodesics_number_of_assignments_before_acceptance << "\n"
    );
}

void OcclusionGeodesicsManager::deleteOccludedTracks(Tracks& tracks, const ros::Time& time)
{
    ROS_DEBUG("Deleting obsolete tracks");

    size_t numDeletedTracks = 0;
    std::vector<Tracks::iterator> tracksToDelete;
    for(Tracks::iterator trackIt = tracks.begin(); trackIt != tracks.end(); ++trackIt) {
        Track::Ptr track = *trackIt;
        switch (track->trackStatus) {
            case Track::MATCHED:
                if (m_likelyOccludedTracks.find(track->id) != m_likelyOccludedTracks.end())
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was likely to be occluded but matched anyway!");
                }
                else if (m_occludedTracks.find(track->id) != m_occludedTracks.end())
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was occluded but matched anyway!");
                    m_occludedTracks.erase(track->id);
                }
                break;
            case Track::MISSED:
            {
                if (m_likelyOccludedTracks.find(track->id) != m_likelyOccludedTracks.end())
                {
                    double secsToOcclusion = ros::Duration(m_likelyOccludedTracks.at(track->id)->occlusionBeginTime - time).toSec();
                    if (secsToOcclusion >= 0)
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
                        if (m_occludedTracks.find(track->id) != m_occludedTracks.end())
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
                if (m_likelyOccludedTracks.find(track->id) != m_likelyOccludedTracks.end())
                {
                    ROS_DEBUG_STREAM("Track " << track->id << " was missed due to a likely occlusion!");
                    m_likelyOccludedTracks.erase(track->id);
                }
                else if (m_occludedTracks.find(track->id) != m_occludedTracks.end())
                {
                    double timeSinceOcclusion = ros::Duration(time - m_occludedTracks.at(track->id)->occlusionEndTime).toSec();
                    if (timeSinceOcclusion > m_allowedDurationForReappearance)
                    {
                        ROS_INFO_STREAM("Track " << track->id << " was occluded but could not be matched for " << timeSinceOcclusion << "seconds. We delete the track!");
                        m_occludedTracks.erase(track->id);
                        tracksToDelete.push_back(trackIt);
                        numDeletedTracks++;
                    }
                    else if (timeSinceOcclusion < 0)
                    {
                        ROS_DEBUG_STREAM("Track " << track->id << " is still occluded for " << fabs(timeSinceOcclusion) << "seconds.");

                    }
                    else
                    {
                        ROS_DEBUG_STREAM("Track " << track->id << " was occluded but could not be matched for " << timeSinceOcclusion << "seconds. We wait!");
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
    if(numDeletedTracks) ROS_DEBUG("%zu track(s) have been deleted!", numDeletedTracks);

    publishMarkerArray();
}

void OcclusionGeodesicsManager::subscribeToLaser(const std::string laserTopic, const std::string laserSegmentationTopic)
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
    m_inputSynchronizers.back()->registerCallback(boost::bind(&OcclusionGeodesicsManager::newLaserscanAndSegmentationAvailable, this, _1,_2,m_numberLaserScanners));

    //Allocate buffer space for new laser
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

void OcclusionGeodesicsManager::newLaserscanAndSegmentationAvailable(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation, unsigned int laserID)
{
    ROS_DEBUG_STREAM("Received new laser scan and segmentation with id " << laserID << " and timestamp " << laserscan->header.stamp.toSec());
    
    // If it is first scan than initialize laser info struct
    if (m_laserInfos.at(laserID).numberMeasurements == 0)
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

Tracks OcclusionGeodesicsManager::manageOcclusionsBeforeDataAssociation(Tracks& tracks, const ros::Time& time, const std::string& trackFrameID)
{
    if (!m_callbackQueue.isEmpty()){
        m_callbackQueue.callAvailable();
    }

    m_currenTrackerTime = time;
    initializeVisualization();
    m_occlusionRegions.clear();

    Tracks detectableTracks, occludedTracks;
    LaserAndSegmentationList laserStructPerLaser;
    for (unsigned int laserID = 0; laserID < m_bufferVector.size(); laserID++)
    {
        m_numCyclesTotal++;

        double smallestTimeDiff;
        LaserAndSegmentationBufferType& buffer = m_bufferVector.at(laserID);
        LaserScanAndSegmentation::Ptr laserStruct = findCorrespondingDataInBuffer(time, buffer, smallestTimeDiff);
        if (!laserStruct)
        {
            m_numCyclesWithMissingSensorData++;
            ROS_WARN_THROTTLE(10.0, "Occlusion Manager could not find suitable data in buffer for timestamp %.3f! Smallest time difference was %.1f ms, but max sync slop is %.1f ms. "
                "%.1f%% of cycles dropped so far due to missing / out-of-sync sensor data! Check laser scan and segmentation timestamps, "
                "and if they are approximately synched with detections!", time.toSec(), 1000.0*smallestTimeDiff, 1000.0*m_maximumSyncSlop, 100.0 * m_numCyclesWithMissingSensorData / m_numCyclesTotal);
            continue;
        }
        else
        {
            if(lookupTransformIntoTrackerFrame(time,laserStruct->laserData->header.frame_id, trackFrameID, laserStruct->transformation))
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

    //Make sure tracks are in clean state
    foreach(Track::Ptr track, tracks){
        //Set track state to MISSED for all tracks
        track->trackStatus = Track::MISSED;
        track->observation.reset();
    }

    const int NOT_IN_VIEW = -1;

    // Loop over set of tracks
    foreach(Track::Ptr track, tracks)
    {
        if (track->numberOfTotalMatches < m_minNumberMatches) {
            detectableTracks.push_back(track);
            continue;
        }
        ROS_DEBUG_STREAM("Starting occlusion analysis for track " << track->id);
        if (m_occludedTracks.find(track->id) != m_occludedTracks.end())
        {

            ROS_DEBUG_STREAM("Track" << track->id <<" is contained in occluded track map.");
            OccludedTrack::Ptr occTrack = m_occludedTracks.at(track->id);
            updateOcclusionGeodesics(occTrack);
            track->trackStatus = Track::OCCLUDED;
            occludedTracks.push_back(occTrack->track);
        }
        else if (m_likelyOccludedTracks.find(track->id) != m_likelyOccludedTracks.end())
        {
            OccludedTrack::Ptr occTrack = m_likelyOccludedTracks.at(track->id);
            ROS_DEBUG_STREAM("Track is contained in likely occluded track map.");
            if (findOccludedTracks(occTrack, m_occlusionRegions, time))
            {
                ROS_DEBUG_STREAM("Track " << track->id << " was likely occluded and is now occluded");
                // TODO do we just want matched updates in this case
                occTrack->stateBeginOcclusion = occTrack->track->state->deepCopy();
                occTrack->numberOfOccludedFrames = 0;
                occTrack->occlusionEndTime = time + ros::Duration(occTrack->distanceToEndOcclusion/occTrack->absoluteVelocity);
                track->numberOfConsecutiveMisses = 0;
                track->trackStatus = Track::OCCLUDED;
                occTrack->occlusionGeodesics.reset(new OcclusionGeodesics(occTrack->track,
                                                                          m_occlusion_geodesics_grid_dimension_forward_meter,
                                                                          m_occlusion_geodesics_grid_dimension_backward_meter,
                                                                          m_occlusion_geodesics_grid_dimension_width_meter,
                                                                          m_occlusion_geodesics_grid_cell_resolution_meter));
                updateOcclusionGeodesics(occTrack);
                m_occludedTracks.insert(std::pair<track_id, OccludedTrack::Ptr> (track->id,occTrack));
                m_likelyOccludedTracks.erase(track->id);
                occludedTracks.push_back(track);
            }
            else if (findLikelyOccludedTracks(occTrack, m_occlusionRegions, time))
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
            OccludedTrack::Ptr occTrack (new OccludedTrack(track, time, m_occlusion_geodesics_number_of_assignments_before_acceptance));
            ROS_DEBUG_STREAM("Track is not yet contained in any occlusion map.");
            if (findOccludedTracks(occTrack, m_occlusionRegions, time))
            {
                ROS_DEBUG_STREAM("Track " << track->id << " is not in any occlusion map, but occluded!");
                occTrack->stateBeginOcclusion = occTrack->track->state->deepCopy();
                occTrack->occlusionEndTime = time + ros::Duration(occTrack->distanceToEndOcclusion/occTrack->absoluteVelocity);
                track->trackStatus = Track::OCCLUDED;
                occTrack->occlusionGeodesics.reset(new OcclusionGeodesics(occTrack->track,
                                                                          m_occlusion_geodesics_grid_dimension_forward_meter,
                                                                          m_occlusion_geodesics_grid_dimension_backward_meter,
                                                                          m_occlusion_geodesics_grid_dimension_width_meter,
                                                                          m_occlusion_geodesics_grid_cell_resolution_meter));
                updateOcclusionGeodesics(occTrack);
                m_occludedTracks.insert(std::pair<track_id, OccludedTrack::Ptr> (track->id,occTrack));
                occludedTracks.push_back(track);
            }
            else if (findLikelyOccludedTracks(occTrack, m_occlusionRegions, time))
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
    }

    visualizeConfidenceMaps();
    visualizeOccludedTracks();
    tracks = detectableTracks;
    return occludedTracks;
}

void OcclusionGeodesicsManager::updateOcclusionGeodesics(OccludedTrack::Ptr occTrack)
{
    occTrack->numberOfOccludedFrames++;
    occTrack->occlusionGeodesics->updateOccludedConfidence(m_occlusionRegions, m_detectionProbabilityVisible, occTrack->numberOfOccludedFrames);
    occTrack->occlusionGeodesics->updatePlausibilityConfidence(occTrack->stateBeginOcclusion, occTrack->track->state, occTrack->numberOfOccludedFrames, m_occlusion_geodesics_motion_variance, m_occlusion_geodesics_plausibility_cut_off);
    occTrack->occlusionGeodesics->updateInertiaConfidence(occTrack->stateBeginOcclusion, occTrack->track->state, m_occlusion_geodesics_inertia_variance);
    occTrack->occlusionGeodesics->updateOcclusionCostMap(m_occlusion_geodesics_infimum_radius);
}

OcclusionGeodesicsManager::LaserScanAndSegmentation::Ptr OcclusionGeodesicsManager::findCorrespondingDataInBuffer(const ros::Time& tracksTime, LaserAndSegmentationBufferType& buffer, double& smallestTimeDiff)
{
    LaserScanAndSegmentation::Ptr bestPtr;
    double smallestAcceptableDiff = m_maximumSyncSlop;
    smallestTimeDiff = std::numeric_limits<double>::infinity();

    LaserAndSegmentationBufferType::size_type bestIdx = 0;
    for (LaserAndSegmentationBufferType::size_type i = 0; i < buffer.size(); i++)
    {
        ros::Duration duration = tracksTime - buffer.at(i)->laserData->header.stamp;
        if(std::abs(duration.toSec()) < smallestTimeDiff) {
            smallestTimeDiff = duration.toSec(); // mainly for logging purposes
        }

        if (fabs(duration.toSec()) < smallestAcceptableDiff)
        {
            bestIdx = i;
            bestPtr = buffer.at(i);
            smallestAcceptableDiff = fabs(duration.toSec());
            if (smallestAcceptableDiff == 0)
                break;
        }

    }
    if (!bestPtr) buffer.rotate(buffer.begin() + bestIdx);

    ROS_DEBUG_STREAM("Size of buffer " << buffer.size() << " and time diff is " << smallestAcceptableDiff << " for track time " << tracksTime.toSec());
    return bestPtr;
}

void OcclusionGeodesicsManager::extractOcclusionRegions(const LaserScanAndSegmentation::Ptr data)
{
    ROS_DEBUG("Extracting segments from scan");
    const sensor_msgs::LaserScan::ConstPtr& laserscan = data->laserData;
    const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation = data->segmentation;

    ROS_DEBUG_STREAM("Extracting polygons for laser with frame_id " << laserscan->header.frame_id << " containing "
                     << laserscan->ranges.size() << " measurements and " << segmentation->segments.size() <<  " segments.");
    OcclusionRegions regions;
    for (size_t i = 0; i < segmentation->segments.size(); i++){
        srl_laser_segmentation::LaserscanSegment segment = segmentation->segments.at(i);
        // Assume measurement indices are ordered

        //Extracting segment data
        if (segment.measurement_indices.size() > 1)
        {
            SegmentInfo::Ptr info(new SegmentInfo);
            info->minAngle = angles::normalize_angle(laserscan->angle_min + laserscan->angle_increment * segment.measurement_indices.front()) ;
            info->maxAngle = angles::normalize_angle(laserscan->angle_min + laserscan->angle_increment * segment.measurement_indices.back()) ;
            info->distAtMinAngle = laserscan->ranges.at(segment.measurement_indices.front());
            info->distAtMaxAngle = laserscan->ranges.at(segment.measurement_indices.back());
            info->label = segment.label;
            info->idxSegmentation = i;

            if (info->distAtMinAngle < (m_occlusionMaxRange- m_selfOcclusionDistance) &&  info->distAtMaxAngle < (m_occlusionMaxRange- m_selfOcclusionDistance))
            {

                OcclusionRegion::Ptr occlusionRegion(new OcclusionRegion);
                occlusionRegion->responsableSegments.push_back(info);
                //Creating occlusion polygon
                Point2D pMinShade = transformToPointInTrackerFrame((info->distAtMinAngle + m_selfOcclusionDistance) * cos(info->minAngle),
                                                                   (info->distAtMinAngle + m_selfOcclusionDistance) * sin(info->minAngle),
                                                                   data->transformation);
                occlusionRegion->occlusionPolygon.add_point(pMinShade);
                Point2D pMinShadeExtended = transformToPointInTrackerFrame(m_occlusionMaxRange * cos(info->minAngle),
                                                                           m_occlusionMaxRange * sin(info->minAngle),
                                                                           data->transformation);
                occlusionRegion->occlusionPolygon.add_point(pMinShadeExtended);
                const double segmentWidthRadians = fabs(info->minAngle - info->maxAngle);
                if (segmentWidthRadians > M_PI_4)
                {

                    int nIntermediatePoints = segmentWidthRadians/M_PI_4;
                    double interval = segmentWidthRadians/(double)nIntermediatePoints;
                    for (int i = 0; i < nIntermediatePoints; i++)
                    {
                        Point2D pIntermediate = transformToPointInTrackerFrame(m_occlusionMaxRange * cos(info->minAngle + (i+1)*interval),
                                                                               m_occlusionMaxRange * sin(info->minAngle + (i+1)*interval),
                                                                               data->transformation);
                        occlusionRegion->occlusionPolygon.add_point(pIntermediate);
                    }
                }
                Point2D pMaxShadeExtended = transformToPointInTrackerFrame(m_occlusionMaxRange * cos(info->maxAngle),
                                                                           m_occlusionMaxRange * sin(info->maxAngle),
                                                                           data->transformation);
                Point2D pMaxShade = transformToPointInTrackerFrame ((info->distAtMaxAngle + m_selfOcclusionDistance) * cos(info->maxAngle),
                                                                    (info->distAtMaxAngle + m_selfOcclusionDistance) * sin(info->maxAngle),
                                                                    data->transformation);
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
                m_occlusionRegions.push_back(occlusionRegion);
            }
        }
    }
}

bool OcclusionGeodesicsManager::findOccludedTracks(OccludedTrack::Ptr occTrack, const OcclusionRegions& occlusionRegions, const ros::Time& time)
{
    ROS_DEBUG_STREAM("Find occlusion");
    bool occlusionFound = false;
    FilterState::Ptr currentState;
    //    if (occTrack->stateBeginOcclusion)
    //        currentState = occTrack->stateBeginOcclusion;
    //    else
    currentState = occTrack->track->state;
    //Transform velocity
    Point2D trackPoint(currentState->xp()(STATE_X_IDX), currentState->xp()(STATE_Y_IDX));
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    velocity(0) = currentState->xp()(STATE_VX_IDX);
    velocity(1) = currentState->xp()(STATE_VY_IDX);

    foreach(OcclusionRegion::Ptr region, occlusionRegions){
        if (boost::geometry::within(trackPoint, region->occlusionPolygon))
        {
            const double extendDistance = 1e6;
            // Store the values for fast access and easy
            // equations-to-code conversion
            double absVelocity = hypot((double)velocity(0),(double)velocity(1));
            absVelocity = std::max(m_minAbsoluteVelocity, absVelocity);
            Linestring2D trackLine;
            Point2D point;
            point.x = trackPoint.x + (velocity(0)/absVelocity * extendDistance);
            point.y = trackPoint.y + (velocity(1)/absVelocity * extendDistance);
            trackLine.push_back(trackPoint);
            trackLine.push_back(point);

            occTrack->absoluteVelocity = absVelocity;

            std::vector<Point2D> intersections;
            boost::geometry::intersection(region->occlusionPolygon, trackLine, intersections);
            if (intersections.size() == 1)
            {
                occTrack->distanceToEndOcclusion = trackPoint.distance(intersections.front());
                //set detection probability and mark track as occluded
                occTrack->track->trackStatus = Track::OCCLUDED;
                occTrack->track->detectionProbability = m_detectionProbabilityOccluded;
                visualizeOcclusionDistance(trackPoint, intersections.front(), occTrack->track->id, m_frameIDTracker);
                occlusionFound = true;
                break;
            }
            else
            {
                ROS_ERROR_STREAM("Track " << occTrack->track->id << " is in occlusion polygon and we found " << intersections.size() << " intersections! Something is definitely going wrong!\nTrack state is" << velocity);
                continue;
            }

        }
    }
    return occlusionFound;
}

bool OcclusionGeodesicsManager::findLikelyOccludedTracks(OccludedTrack::Ptr occTrack, const OcclusionRegions& occlusionRegions, const ros::Time& time)
{
    ROS_DEBUG_STREAM("Find likely occlusion");
    bool likelyOccludedFound = false;
    FilterState::Ptr currentState;
    //    if (occTrack->stateBeginOcclusion)
    //        currentState = occTrack->stateBeginOcclusion;
    //    else
    currentState = occTrack->track->state;
    Point2D trackPoint(currentState->xp()(STATE_X_IDX), currentState->xp()(STATE_Y_IDX));

    //Transform velocity
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    velocity(0) = currentState->xp()(STATE_VX_IDX);
    velocity(1) = currentState->xp()(STATE_VY_IDX);

    foreach(OcclusionRegion::Ptr region, occlusionRegions) {
        foreach(Polygon2D polygon, region->occlusionRiskPolygons) {
            if (boost::geometry::within(trackPoint, polygon))
            {
                const double extendDistance = 1e6;
                // Store the values for fast access and easy
                // equations-to-code conversion
                double absVelocity = hypot((double)velocity(0),(double)velocity(1));
                absVelocity = std::max(m_minAbsoluteVelocity, absVelocity);
                Linestring2D trackLine;
                Point2D point;
                point.x = trackPoint.x + (velocity(0)/absVelocity * extendDistance);
                point.y = trackPoint.y + (velocity(1)/absVelocity * extendDistance);
                trackLine.push_back(trackPoint);
                trackLine.push_back(point);

                occTrack->absoluteVelocity = absVelocity;

                std::vector<Point2D> intersections;
                boost::geometry::intersection(region->occlusionPolygon, trackLine, intersections);
                if (intersections.size() == 2)
                {
                    occTrack->distanceToBeginOcclusion = trackPoint.distance(intersections.at(0));
                    occTrack->distanceToEndOcclusion = trackPoint.distance(intersections.at(1));
                    if (occTrack->distanceToBeginOcclusion > occTrack->distanceToEndOcclusion)
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
                else if (intersections.size() == 0)
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

bool OcclusionGeodesicsManager::lookupTransformIntoTrackerFrame(ros::Time stamp, const std::string& sourceFrame ,const std::string& targetFrame, Eigen::Affine3d& resultingTransform)
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

Point2D OcclusionGeodesicsManager::transformToPointInTrackerFrame(const double x, const double y, Eigen::Affine3d& transform)
{
    Eigen::Vector3d pointLaser = Eigen::Vector3d::Zero();
    pointLaser(0) = x;
    pointLaser(1) = y;


    // Transform pose and covariance into our reference frame, still in 3D
    Eigen::Vector3d pointTracker = transform * pointLaser;

    return Point2D(pointTracker(0), pointTracker(1));
}

double OcclusionGeodesicsManager::calculateOcclusionProbability(const double distanceToShade)
{
    // LINEAR COST FUNFCTION
    if (distanceToShade > m_neighborPolygonWidth)
    {
        return 0.0;
    }
    else
    {
        return (m_neighborPolygonWidth-distanceToShade)/m_neighborPolygonWidth;
    }
}

void OcclusionGeodesicsManager::setDetectionProbability(const double occlusionProbability, const Track::Ptr track)
{
    track->detectionProbability = m_detectionProbabilityOccluded + (m_detectionProbabilityVisible - m_detectionProbabilityOccluded) * (1-occlusionProbability);
    //  if (track->detectionProbability < 0.5)
    //    track->trackStatus = Track::OCCLUDED;
}


Pairings OcclusionGeodesicsManager::occludedTrackAssociation(Tracks tracks, Observations observations, const ros::Time& time)
{
    //    assert(tracks.size() == m_occludedTracks.size());
    //
    // Get unmatched observations
    Observations unmatchedObservations;
    foreach(Observation::Ptr observation, observations){
        if (!observation->matched)
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
            double current_cost = it_track->second->occlusionGeodesics->getConfidenceAt(observation->z);
            if (m_visualizationEnabled){
                visualization_msgs::MarkerPtr marker = it_track->second->occlusionGeodesics->getSelectedCellMarker(observation->z, m_frameIDTracker, m_currenTrackerTime, observation->id);
                if (marker){
                    m_currentMarkers.markers.push_back(*marker);
                }
            }
            if (std::isnan(current_cost) || std::isinf(current_cost))
                current_cost = BIG_COST;
            reassignmentCost(i_track,i_ob) = current_cost;
            if (current_cost < BIG_COST)
                foundAtLeastOnePossibleAssociation = true;
            i_track++;
        }
    }

    if (foundAtLeastOnePossibleAssociation)
    {
        LAPSolver<double> linearAssignmentProblem;
        Eigen::VectorXi trackAssignments, obsAssignments;

        ROS_DEBUG_STREAM("Cost Matrix " << reassignmentCost);

        double totalCost = linearAssignmentProblem.calculateAssignment(reassignmentCost, trackAssignments, obsAssignments);

        unsigned int i_track = 0;
        for (OccludedTrackMap::iterator it_track=m_occludedTracks.begin(); it_track != m_occludedTracks.end(); ++it_track)
        {
            OccludedTrack::Ptr& occTrack = it_track->second;
            //Make sure
            occTrack->track->observation.reset();
            switch (occTrack->track->trackStatus){
                case Track::MISSED:
                    occTrack->track->numberOfConsecutiveMisses++;
                    break;
                case Track::OCCLUDED:
                    occTrack->track->numberOfConsecutiveOcclusions++;
                    break;
            }
            double currentCost = reassignmentCost(i_track, obsAssignments(i_track));
            if (currentCost < BIG_COST)
            {
                Observation::Ptr& assignendObservation = unmatchedObservations.at(obsAssignments(i_track));
                occTrack->acceptedAssignements.push_back(assignendObservation);
                assignendObservation->matched = true;
                if(occTrack->acceptedAssignements.size() == occTrack->acceptedAssignements.capacity())
                {
                    bool orientationTooDifferent = false;
                    bool observationTooFar = false;

                    if (m_occlusion_geodesics_number_of_assignments_before_acceptance > 1)
                    {
                        Observation::Ptr& ob1 = occTrack->acceptedAssignements.front();
                        Observation::Ptr& ob2 = occTrack->acceptedAssignements.back();
                        ObsVector diff = ob2->z - ob1->z;
                        double obsOrientation = atan2(diff(1), diff(0));
                        double trackOrientation = atan2(occTrack->stateBeginOcclusion->x()(STATE_VY_IDX),occTrack->stateBeginOcclusion->x()(STATE_VX_IDX));
                        if (fabs(angles::shortest_angular_distance(obsOrientation, trackOrientation)) > m_occlusion_geodesics_allowed_orientation_difference_for_acceptance)
                        {
                            orientationTooDifferent = true;
                            ROS_DEBUG_STREAM("Orientation of occluded track " << occTrack->track->id << " is to different for assignment!");
                        }

                        for(boost::circular_buffer<Observation::Ptr>::iterator it = occTrack->acceptedAssignements.begin(); it != occTrack->acceptedAssignements.end()-1 ; ++it)
                        {
                            Observation::Ptr& ob1 = *it;
                            Observation::Ptr& ob2 = *(it+1);
                            ObsVector diff = ob1->z.head(OBS_DIM)-ob2->z.head(OBS_DIM);
                            if (diff.squaredNorm() > 0.25){
                                observationTooFar = true;
                            }
                        }
                    }
                    if (!observationTooFar && !orientationTooDifferent){
                        foreach(Observation::Ptr ob, occTrack->acceptedAssignements){
                            Pairing::Ptr pairing( new Pairing );
                            Track::Ptr track = occTrack->track;
                            pairing->track = track;
                            pairing->observation = ob;
                            pairing->validated = false;
                            //visualizePossibleOcclusionAssignments(pairing, currentCost);
                            // Calculate innovation v and inverse of innovation covariance S
                            pairing->v = pairing->observation->z - pairing->track->state->zp();
                            //todo wie stark das update uebernommen werden soll
                            ObsMatrix S = ObsMatrix::Zero();
                            if (!m_scaleUpdateWithCost)
                                S = pairing->track->state->H() * pairing->track->state->Cp() * pairing->track->state->H().transpose() + pairing->observation->R;
                            else
                                S = pairing->track->state->H() * pairing->track->state->Cp() * pairing->track->state->H().transpose() + (pairing->observation->R*currentCost);
                            Eigen::FullPivLU<ObsMatrix> lu(S);
                            double ln_det_S = 0;
                            ln_det_S = log(lu.determinant());
                            // Calculate inverse of innovation covariance if possible
                            if (ln_det_S > MATRIX_LN_EPS)
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
                            if(!pairing->singular) {
                                // Store in list of compatible pairings
                                compatiblePairings.push_back(pairing);
                                ROS_DEBUG_STREAM("Current cost = " << currentCost);
                                ROS_INFO_STREAM("Occlusion Geodesics reassignment of track " << track->id << " to ob "<< pairing->observation->id <<  ": \n" <<
                                                "Current time:" << time.toSec() << "\tOcclusionStartTime:" << occTrack->occlusionBeginTime.toSec() << "\tOcclusionEndTime:" << occTrack->occlusionEndTime.toSec() << "\n" <<
                                                "Occlusion therefore took " << ros::Duration(time-occTrack->occlusionBeginTime).toSec() << " expected were " << ros::Duration(occTrack->occlusionEndTime-occTrack->occlusionBeginTime).toSec()<< "\n" <<
                                                "Current track prediction: (" << track->state->xp()(0) << track->state->xp()(1) << ")\n" <<
                                                "Associated observation: (" << pairing->observation->z(0) << pairing->observation->z(1) << ")\n" <<
                                                "Gap begin to Observation: " << (occTrack->stateBeginOcclusion->x().head(2)-pairing->observation->z).norm() << "\n" <<
                                                "Normalized occlusion cost: " << currentCost/(double)occTrack->numberOfOccludedFrames << "\n" <<
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
                            else
                            {
                                pairing->observation.reset();
                            }
                        }
                    }
                }
            }
            i_track++;
        }
        if (compatiblePairings.size() > 0)
            ROS_DEBUG_STREAM("Reassigned " << compatiblePairings.size()/ m_occlusion_geodesics_number_of_assignments_before_acceptance << " occluded tracks.");
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
            OccludedTrack::Ptr& occTrack = it_track->second;
            occTrack->track->observation.reset();
            switch (occTrack->track->trackStatus){
                case Track::MISSED:
                    occTrack->track->numberOfConsecutiveMisses++;
                    break;
                case Track::OCCLUDED:
                    occTrack->track->numberOfConsecutiveOcclusions++;
                    break;
            }
        }
    }
    return compatiblePairings;
}



///*****************************************************************
/// VISUALIZATION STUFF
///*****************************************************************

void OcclusionGeodesicsManager::initializeVisualization()
{
    if (m_visualizationEnabled)
    {
        foreach(visualization_msgs::Marker& marker, m_currentMarkers.markers){
            marker.action = visualization_msgs::Marker::DELETE;
        }
        m_visualizationPublisher.publish(m_currentMarkers);
        m_currentMarkers.markers.clear();
    }
}

void OcclusionGeodesicsManager::publishMarkerArray()
{
    if (m_visualizationEnabled)
    {
        m_visualizationPublisher.publish(m_currentMarkers);
    }
}

void OcclusionGeodesicsManager::visualizeConfidenceMaps()
{
    if (m_visualizationEnabled && m_occludedTracks.size() > 0)
    {
        for (OccludedTrackMap::iterator it_track=m_occludedTracks.begin(); it_track != m_occludedTracks.end(); ++it_track)
        {
            OccludedTrack::Ptr& occTrack = it_track->second;
            if (occTrack->occlusionGeodesics)
            {
                m_currentMarkers.markers.push_back(*(occTrack->occlusionGeodesics->getConfidenceMapMarker(m_frameIDTracker, m_currenTrackerTime, occTrack->track->id, occTrack->numberOfOccludedFrames)));
                m_currentMarkers.markers.push_back(*(occTrack->occlusionGeodesics->getInertialConfidenceMapMarker(m_frameIDTracker, m_currenTrackerTime, occTrack->track->id)));
                m_currentMarkers.markers.push_back(*(occTrack->occlusionGeodesics->getOcclusionConfidenceMapMarker(m_frameIDTracker, m_currenTrackerTime, occTrack->track->id)));
                m_currentMarkers.markers.push_back(*(occTrack->occlusionGeodesics->getPlausibilityConfidenceMapMarker(m_frameIDTracker, m_currenTrackerTime, occTrack->track->id)));
            }
        }
    }
}

void OcclusionGeodesicsManager::visualizeOcclusionDistance(Point2D& trackPosition, Point2D& intersection, const unsigned int trackID, const std::string& frame_id)
{
    if (m_visualizationEnabled)
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

void OcclusionGeodesicsManager::visualizeOcclusionPolygons(LaserScanAndSegmentation::Ptr laserData)
{
    if (m_visualizationEnabled)
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


        marker.header.frame_id = markerRisk.header.frame_id = m_frameIDTracker;
        OcclusionRegions& regions = m_occlusionRegions;
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

void OcclusionGeodesicsManager::visualizeOccludedTracks()
{
    if (m_visualizationEnabled && m_occludedTracks.size() > 0)
    {
        visualization_msgs::Marker markerOccludedTracks, markerPrediction;
        //Common settings
        markerOccludedTracks.header.frame_id = markerPrediction.header.frame_id = m_frameIDTracker;
        markerOccludedTracks.header.stamp = markerPrediction.header.stamp = m_currenTrackerTime;
        markerOccludedTracks.action = markerPrediction.action =visualization_msgs::Marker::ADD;
        markerOccludedTracks.ns = markerPrediction.ns ="occlusion_begin_state";
        //marker for begin off occlusion cube
        markerOccludedTracks.id = 1 ;
        markerOccludedTracks.type =  visualization_msgs::Marker::CUBE_LIST;
        markerOccludedTracks.color.g = 1.0f;
        markerOccludedTracks.color.a = 1.0;
        markerOccludedTracks.scale.x = 0.2;
        markerOccludedTracks.scale.y = 0.2;
        markerOccludedTracks.scale.z = 0.2;
        //marker for showing connection between current prediction and begin of occlusion
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

void OcclusionGeodesicsManager::visualizePossibleOcclusionAssignments(Pairing::Ptr pairing, const double cost)
{
    if (m_visualizationEnabled && pairing)
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

void OcclusionGeodesicsManager::visualizeOcclusionAssignments(Pairings pairings)
{
    if (m_visualizationEnabled)
    {
        if (pairings.size() > 0)
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

            foreach(Pairing::Ptr pairing, pairings){

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

