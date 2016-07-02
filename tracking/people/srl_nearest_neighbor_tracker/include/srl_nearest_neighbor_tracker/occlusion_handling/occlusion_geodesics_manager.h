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

#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_GEODESICS_MANAGER_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_GEODESICS_MANAGER_H_

/// Boost related includes
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

/// ROS basic includes
#include <ros/ros.h>
#include <tf/transform_listener.h>

/// ROS messages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <srl_laser_segmentation/LaserscanSegmentation.h>

/// Nearest neighbor includes
#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/data/track.h>
#include <srl_nearest_neighbor_tracker/data/pairing.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_manager.h>
#include <srl_nearest_neighbor_tracker/data/polygon_boost.h>
#include <srl_nearest_neighbor_tracker/data/linestring_boost.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_geodesics.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_data_utils.h>



namespace srl_nnt {

class OcclusionGeodesicsManager : public OcclusionManager
{
public:
    OcclusionGeodesicsManager();

    /// Public functions for occlusion handler interface
    virtual void initializeOcclusionManager(const ros::NodeHandle& nodehandle,const ros::NodeHandle& privateNodeHandle);
    virtual Tracks manageOcclusionsBeforeDataAssociation(Tracks& tracks,const ros::Time& time, const std::string& trackFrameID);
    virtual void deleteOccludedTracks(Tracks& tracks,const ros::Time& time);
    virtual Pairings occludedTrackAssociation(const Tracks tracks,const Observations observations, const ros::Time& time);

private:
    struct OccludedTrack {
        Track::Ptr track;
        FilterState::Ptr stateBeginOcclusion;
        ros::Time occlusionEndTime;
        ros::Time occlusionBeginTime;
        double distanceToBeginOcclusion;
        double distanceToEndOcclusion;
        double absoluteVelocity;
        int laserID;
        unsigned int numberOfOccludedFrames;
        unsigned int numberConsecutiveNonOcclusions;
        OcclusionGeodesics::Ptr occlusionGeodesics;
        boost::circular_buffer<Observation::Ptr> acceptedAssignements;

        OccludedTrack(Track::Ptr occTrack,const ros::Time& time, unsigned int bufferLength)
        {
            numberConsecutiveNonOcclusions=0;
            numberOfOccludedFrames=0;
            absoluteVelocity = 0.0;
            distanceToBeginOcclusion = 0.0;
            distanceToEndOcclusion =0.0;
            laserID =0;
            acceptedAssignements.set_capacity(bufferLength);
            occlusionBeginTime = time;
            track = occTrack;
        }
        typedef boost::shared_ptr<OccludedTrack> Ptr;
    };

    typedef std::map<track_id, OccludedTrack::Ptr> OccludedTrackMap;
    typedef std::map<track_id, OccludedTrack::Ptr> TrackMap;
    OccludedTrackMap m_occludedTracks;
    OccludedTrackMap m_likelyOccludedTracks;
    TrackMap m_outOfViewTracks;

    /// struct to save some important data for each laser device
    struct LaserInfo
    {
        double minAngle;
        double maxAngle;
        double angleIncrement;
        unsigned int numberMeasurements;
        std::string frame_id;
        typedef boost::shared_ptr<LaserInfo> Ptr;
    };

    /// container type to save laser scanner device infos
    typedef std::vector<LaserInfo> LaserInfos;
    LaserInfos m_laserInfos;

    struct LaserScanAndSegmentation
    {
        sensor_msgs::LaserScan::ConstPtr laserData;
        srl_laser_segmentation::LaserscanSegmentation::ConstPtr segmentation;
        Eigen::Affine3d transformation;

        unsigned int laserID;
        typedef boost::shared_ptr<LaserScanAndSegmentation> Ptr;
    };

    typedef boost::circular_buffer<LaserScanAndSegmentation::Ptr> LaserAndSegmentationBufferType;
    std::vector<LaserAndSegmentationBufferType> m_bufferVector;
    typedef std::vector<LaserScanAndSegmentation::Ptr> LaserAndSegmentationList;
    LaserAndSegmentationList m_laserDataVector;

    tf::TransformListener m_transformListener;

    /// Node handles
    ros::NodeHandle m_nodeHandle, m_privateNodeHandle;

    bool m_visualizationEnabled;

    /// Publisher for visualization
    ros::Publisher m_visualizationPublisher;

    /// Subscribers
    /// For synchronization of input laserscans and segmentations
    typedef boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > LaserscanSubscriber_t;
    typedef boost::shared_ptr<message_filters::Subscriber<srl_laser_segmentation::LaserscanSegmentation> > SegmentationSubscriber_t;
    std::vector<LaserscanSubscriber_t> m_laserscanSubscribers;
    std::vector<SegmentationSubscriber_t> m_segmentationSubscribers;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::LaserScan, srl_laser_segmentation::LaserscanSegmentation> SyncPolicy_t;
    typedef message_filters::Synchronizer<SyncPolicy_t> Synchronizer_t;
    typedef boost::shared_ptr<Synchronizer_t> InputSynchronizer_t;
    std::vector<InputSynchronizer_t> m_inputSynchronizers;
    ros::CallbackQueue m_callbackQueue;

    double m_detectionProbabilityOccluded;
    double m_detectionProbabilityVisible;
    double m_selfOcclusionDistance;
    double m_allowedDurationForReappearance;
    double m_occlusionMaxRange;
    double m_neighborPolygonWidth;
    unsigned int m_numberLaserScanners;
    double m_maximumSyncSlop;
    bool m_scaleUpdateWithCost;
    int m_minNumberMatches;
    ros::Time m_currenTrackerTime;
    OcclusionRegions m_occlusionRegions;

    int m_numCyclesTotal, m_numCyclesWithMissingSensorData; // for statistics w.r.t. time synchronization

    unsigned int m_visNumberPolygons;
    unsigned int m_visNumberRiskPolygons;
    visualization_msgs::MarkerArray m_currentMarkers;

    typedef std::map<SegmentInfo::Ptr, int> OcclusionSegmentMap;

    /// Subscribe to a new laser scanner py passing topic names of scan and corresponding segmentation
    void subscribeToLaser(const std::string laserTopic, const std::string laserSegmentationTopic);

    /// callback function for new laser data and segmentation
    void newLaserscanAndSegmentationAvailable(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation, unsigned int laserID);

    /// find corresponding segment information for current track time
    LaserScanAndSegmentation::Ptr findCorrespondingDataInBuffer(const ros::Time& tracksTime, LaserAndSegmentationBufferType& buffer, double& smallestTimeDiff);

    /// transform a track into the sensor coordinate system
    void transformTrackToTrackerFrame(Track::Ptr track, Eigen::Affine3d& transform, Eigen::Vector3d& meanTrackLF, Eigen::Matrix3d& covTrackLF);

    /// find the corresponding transform from track to laser for current time
    bool lookupTransformIntoTrackerFrame(ros::Time stamp, const std::string& sourceFrame ,const std::string& targetFrame, Eigen::Affine3d& resultingTransform);

    /// extract important data from segmentation -- Assumption: segment indices are ordered
    void extractOcclusionRegions(const LaserScanAndSegmentation::Ptr data);

    /// calculate the index from a passed angle in radians
    int getIndexFromAngle(double angle, unsigned int laserID);

    void visualizeOcclusionDistance(Point2D& trackPosition, Point2D& intersection, const unsigned int trackID, const std::string& frame_id);
    void visualizeOcclusionPolygons(LaserScanAndSegmentation::Ptr laserData);
    bool findLikelyOccludedTracks(OccludedTrack::Ptr occTrack,const OcclusionRegions& occlusionRegions, const ros::Time& time);
    bool findOccludedTracks(OccludedTrack::Ptr occTrack,const OcclusionRegions& occlusionRegions, const ros::Time& time);
    double calculateOcclusionProbability(const double distanceToShade);
    void setDetectionProbability(const double occlusionProbability, const Track::Ptr track);
    double calcAssociationCost(OccludedTrack::Ptr track, Observation::Ptr observation, const ros::Time& time);
    void visualizeOcclusionAssignments(Pairings pairings);
    bool checkTrackForObservability(const Eigen::Vector3d& trackMean,unsigned int laserID);
    void initializeVisualization();
    void publishMarkerArray();
    void visualizePossibleOcclusionAssignments(Pairing::Ptr pairing, const double cost);
    void visualizeOccludedTracks();
    Point2D transformToPointInTrackerFrame(const double x, const double y, Eigen::Affine3d& transform);
    void visualizeConfidenceMaps();
    void updateOcclusionGeodesics(OccludedTrack::Ptr occTrack);

    /// member variables for deletion of missed tracks
    int m_MAX_MISSES_BEFORE_DELETION ;
    int m_MAX_MISSES_BEFORE_DELETION_OF_MATURE_TRACK ;
    int m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES ;

    double m_occlusion_geodesics_inertia_variance;
    double m_occlusion_geodesics_motion_variance;
    double m_occlusion_geodesics_plausibility_cut_off;
    double m_occlusion_geodesics_grid_dimension_forward_meter;
    double m_occlusion_geodesics_grid_dimension_backward_meter;
    double m_occlusion_geodesics_grid_dimension_width_meter;

    double m_occlusion_geodesics_grid_cell_resolution_meter;
    int m_occlusion_geodesics_infimum_radius;
    double m_minAbsoluteVelocity;
    int m_occlusion_geodesics_number_of_assignments_before_acceptance;
    double m_occlusion_geodesics_allowed_orientation_difference_for_acceptance;

};
}


#endif /* _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_GEODESICS_MANAGER_H_ */
