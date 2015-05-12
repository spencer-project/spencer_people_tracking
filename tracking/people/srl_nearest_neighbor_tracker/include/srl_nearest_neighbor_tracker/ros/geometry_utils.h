/* Created on: May 07, 2014. Author: Timm Linder */
#ifndef _GEOMETRY_UTILS_H
#define _GEOMETRY_UTILS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <string>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <Eigen/Geometry>


namespace srl_nnt
{

class GeometryUtils
{
public:
    /// Constructor.
    GeometryUtils();

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


private:
    tf::TransformListener m_transformListener;
};


} // end of namespace geometry


#endif // _GEOMETRY_UTILS_H