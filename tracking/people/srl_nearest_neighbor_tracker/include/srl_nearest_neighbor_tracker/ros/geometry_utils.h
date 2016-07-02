/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015-2016, Timm Linder, Social Robotics Lab, University of Freiburg
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

#ifndef _GEOMETRY_UTILS_H
#define _GEOMETRY_UTILS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <string>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <spencer_tracking_msgs/DetectedPersons.h>

#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <Eigen/Geometry>


namespace srl_nnt
{

class GeometryUtils
{
public:
    /// Constructor.
    GeometryUtils();

    /// Returns the singleton instance, if any
    static GeometryUtils& getInstance() {
        ROS_ASSERT(s_instance != NULL);
        return *s_instance;
    }

    /// Returns the ID of the fixed world frame used for tracking.
    std::string getWorldFrame();

    /// Lookup the transform from the given source frame into the fixed world frame, for the given timestamp. Returns an Eigen::Affine3d transform.
    bool lookupTransformIntoWorldFrame(ros::Time stamp, const std::string& sourceFrame, Eigen::Affine3d& resultingTransform);

    /// Initialize a PoseWithCovariance and TwistWithCovariance message from the given mean vector and covariance matrix.
    void meanAndCovarianceToPoseAndTwist(const StateVector& x, const StateMatrix& C, geometry_msgs::PoseWithCovariance& pose, geometry_msgs::TwistWithCovariance& twist);

    // Extracts mean and covariance from a given pose, also taking the transform from the given source frame into the fixed world frame into account.
    void poseToMeanAndCovariance(const geometry_msgs::PoseWithCovariance& pose, ObsVector& x, ObsMatrix& C, const Eigen::Affine3d& transformToApply);

    /// Checks if a pose has reasonable values (no NaN values, coordinates not extremely large, covariance matrix X/Y positive semi-definite, etc.)
    bool posePassesSanityCheck(const geometry_msgs::PoseWithCovariance& pose, bool checkOrientation = true);

    /// Converts the ROS DetectedPersons messages into Observation instances, and transforms them into our world frame.
    void convertDetectedPersonsToObservations(const spencer_tracking_msgs::DetectedPersons::ConstPtr& detectedPersons, Observations& observations);

    /// Converts Observations into ROS DetectedPersons messages (for debugging usually).
    void convertObservationsToDetectedPersons(const double currentTime, const Observations& observations, spencer_tracking_msgs::DetectedPersons& detectedPersons);

private:
    tf::TransformListener m_transformListener;
    static GeometryUtils* s_instance;
};


} // end of namespace geometry


#endif // _GEOMETRY_UTILS_H