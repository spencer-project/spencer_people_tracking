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

#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_LASER_SHADE_OCCLUSION_MANAGER_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_LASER_SHADE_OCCLUSION_MANAGER_H_

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

#include <srl_laser_segmentation/LaserscanSegmentation.h>
#include <srl_nearest_neighbor_tracker/data/track.h>
#include <srl_nearest_neighbor_tracker/base/multivariate_normal_sampler.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_manager.h>


namespace srl_nnt {

class LaserShadeOcclusionManager : public OcclusionManager
{
public:
    LaserShadeOcclusionManager();

    /// Public functions for occlusion handler interface
    virtual void initializeOcclusionManager(const ros::NodeHandle& nodehandle,const ros::NodeHandle& privateNodeHandle);
    virtual Tracks manageOcclusionsBeforeDataAssociation(Tracks& tracks,const ros::Time& time, const std::string& trackFrameID);
    virtual void deleteOccludedTracks(Tracks& tracks,const ros::Time& time);
    virtual Pairings occludedTrackAssociation(Tracks tracks, Observations observations, const ros::Time& time) { return Pairings();}

private:
    ///Basic parameters fo occlusion handler
    unsigned int m_numberSamples;
    double m_addtionalTimeFactor;
    double m_occlusionMaxRange;

    // helper struct-type to save 2D points easily
    struct Point2D {
        double x;
        double y;
    };


    /// struct for saving interesting data for occlusion handling for segments
    struct SegmentInfo{
        double minAngle;
        double maxAngle;
        double minDist;
        double maxDist;
        unsigned int label;
        unsigned int idx;
        /// for easier readability
        typedef boost::shared_ptr<SegmentInfo> Ptr;
    };

    /// container type to save segment infos
    typedef std::vector<SegmentInfo::Ptr> SegmentDataPolar;

    struct OccludedTrack{
        Track::Ptr track;
        ros::Time occlusionTime;
        SegmentInfo::Ptr occlusionSegment;
        StateVector beginOcclusionState;
        StateMatrix beginOcclusionCov;
        StateVector endOcclusionState;
        StateMatrix endOcclusionCov;
        /// for easier readability
        typedef boost::shared_ptr<OccludedTrack> Ptr;
    };


    /// struct to save some important data for each laser device
    struct LaserInfo{
        double minAngle;
        double maxAngle;
        double angleIncrement;
        unsigned int numberMeasurements;
    };
    /// container type to save laser scanner device infos
    typedef std::vector<LaserInfo> LaserInfos;
    LaserInfos m_laserInfos;


    tf::TransformListener m_transformListener;

    MultiVariateNormalDistribution<double>::Ptr m_sampler;


    /// Node handles
    ros::NodeHandle m_nodeHandle, m_privateNodeHandle;

    bool m_visualizationEnabled;
    /// Publisher for visualization
    ros::Publisher m_visualizationPublisher;
    /// point container type for visualization of samples
    typedef std::vector<geometry_msgs::Point> Samples;
    /// container for samples depending on their classification
    Samples m_visible;
    Samples m_occluded;
    Samples m_outOfView;

    /// Subscribers
    /// For synchronization of input laserscans and segmentations
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > m_laserscanSubscriber;
    boost::shared_ptr<message_filters::Subscriber<srl_laser_segmentation::LaserscanSegmentation> > m_segmentationSubscriber;


    typedef message_filters::sync_policies::ExactTime<sensor_msgs::LaserScan, srl_laser_segmentation::LaserscanSegmentation> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> m_inputSynchronizer;

    ///local information for tracks
    typedef std::map<track_id, OccludedTrack::Ptr> OccludedTrackMap;
    OccludedTrackMap m_occludedTracks;
    OccludedTrackMap m_redetectableTracks;

    double m_detectionProbabilityOccluded;
    double m_detectionProbabilityVisible;
    unsigned int m_numberLaserScanners;


    typedef std::pair<sensor_msgs::LaserScan::ConstPtr,srl_laser_segmentation::LaserscanSegmentation::ConstPtr> LaserScanAndSegmentation;
    typedef boost::circular_buffer<LaserScanAndSegmentation> BufferType;
    std::vector<BufferType> m_bufferVector;

    typedef std::map<SegmentInfo::Ptr, int> OcclusionSegmentMap;

    /// Subscribe to a new laser scanner py passing topic names of scan and corresponding segmentation
    void subscribeToLaser(const std::string laserTopic, const std::string laserSegmentationTopic);

    /// callback function for new laser data and segmentation
    void newLaserscanAndSegmentationAvailable(const sensor_msgs::LaserScan::ConstPtr& laserscan, const srl_laser_segmentation::LaserscanSegmentation::ConstPtr& segmentation, unsigned int laserID);

    /// find corresponding segment information for current track time
    int findCorrespondingDataInBuffer(const ros::Time& tracksTime, BufferType& buffer);

    /// create samples from 3D gaussian distribution
    void sampleFromDistribution(Eigen::Vector3d& mean, Eigen::Matrix3d& cov, unsigned int n_samples, Eigen::Matrix3Xd& samples);

    /// transform a track into the sensor coordinate system
    void transformTrackToSensorFrame(Track::Ptr track, Eigen::Affine3d& transform, Eigen::Vector3d& meanTrackLF, Eigen::Matrix3d& covTrackLF);

    /// find the corresponding transform from track to laser for current time
    bool lookupTransformIntoSensorFrame(ros::Time stamp, const std::string& sourceFrame ,const std::string& targetFrame, Eigen::Affine3d& resultingTransform);

    /// extract important data from segmentation -- Assumption: segment indices are ordered
    void extractSegmentData(LaserScanAndSegmentation& data, unsigned int laserID, SegmentDataPolar& segmentData);

    /// calculate the index from a passed angle in radians
    int getIndexFromAngle(double angle, unsigned int laserID);

    /// calculate occlusion probability for the passed samples
    double getOcclusionProbabilityForSamples(Eigen::Matrix3Xd& samples, LaserScanAndSegmentation& laserData, SegmentDataPolar& segmentData,unsigned int laserID, OcclusionSegmentMap& responsibleSegments);

    /// depending on the direction of tracked person find the most important segment for the occlusion if more segments are responsible for occlusion
    void identifyMostImportantOcclusionSegment(Eigen::Vector3d& trackVelocity, OcclusionSegmentMap& occlusionSegments, SegmentInfo::Ptr& importantSegment);

    /// get velocity data as 3d vector
    void transformVelocityToSensorFrame(Track::Ptr track, Eigen::Affine3d& transform, Eigen::Vector3d& velocityLF);

    /// calculate the intersection of the predicted track and the end of the occlusion shade
    double calculateIntersectionWithOcclusionShade(SegmentInfo::Ptr segmentInfo, LaserScanAndSegmentation& laserData, Eigen::Vector3d& trackPosition, Eigen::Vector3d& trackVelocity, Eigen::Vector3d& intersection);

    /// helper function to find intersection between line segment and polygon
    bool calculateIntersectionFromLineAndPolygon(const std::vector<Point2D> &line, const std::vector<Point2D> &polygon, Eigen::Vector3d& result);

    /// helper function to find intersection between two line segments
    bool calculateIntersectionFromTwoLineSegments(const double xt1, const double yt1, const double xt2, const double yt2, const double xs1, const double ys1, const double xs2, const double ys2, Eigen::Vector3d& result);

    /// visualization method for samples as points
    void visualizeSamples(LaserScanAndSegmentation& laserData);

    /// visualization method for the predicted distance in shade as arrow
    void visualizeOcclusionDistance(Eigen::Vector3d& trackPosition, Eigen::Vector3d& intersection, LaserScanAndSegmentation& laserData, const unsigned int trackID);

    /// delete old occlusion markers
    void deleteOcclusionDistanceMarker(LaserScanAndSegmentation& laserData, const unsigned int trackID);

    /// member variables for deletion of missed tracks
    int m_MAX_MISSES_BEFORE_DELETION ;
    int m_MAX_MISSES_BEFORE_DELETION_OF_MATURE_TRACK ;
    int m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES ;



};
}





#endif /* _SRL_NEAREST_NEIGHBOR_TRACKER_LASER_SHADE_OCCLUSION_MANAGER_H_ */
