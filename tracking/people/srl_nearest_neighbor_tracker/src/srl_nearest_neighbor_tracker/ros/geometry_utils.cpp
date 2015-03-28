#include <srl_nearest_neighbor_tracker/ros/geometry_utils.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <Eigen/Eigenvalues>


namespace srl_nnt
{

GeometryUtils::GeometryUtils() : m_transformListener()
{
}


std::string GeometryUtils::getWorldFrame()
{
    return srl_nnt::Params::get<std::string>("world_frame", "odom");
}


bool GeometryUtils::lookupTransformIntoWorldFrame(ros::Time stamp, const string& sourceFrame, Eigen::Affine3d& resultingTransform)
{
    tf::StampedTransform tfTransform;
    const string targetFrame = getWorldFrame();

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
    return true;
}


void GeometryUtils::meanAndCovarianceToPoseAndTwist(const StateVector& x, const StateMatrix& C, geometry_msgs::PoseWithCovariance& pose, geometry_msgs::TwistWithCovariance& twist)
{
    // Some constants for determining the pose
    const double AVERAGE_ROTATION_VARIANCE = pow(10.0 / 180 * M_PI, 2); // FIXME: determine from vx, vy?
    const double INFINITE_VARIANCE = 9999999; // should not really use infinity here because then the covariance matrix cannot be rotated (singularities!)

    assert(x.size() % 2 == 0);
    const int numAxes = x.size() / 2; // either 2 or 3
    const double vx = x(numAxes+0), vy = x(numAxes+1);

    // Set pose (=position + orientation)
    pose.pose.position.x = x(0);
    pose.pose.position.y = x(1);
    pose.pose.position.z = numAxes > 2 ? x(2) : 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(vy, vx)); // determine orientation from current velocity estimate

    pose.covariance.fill(0.0);
    pose.covariance[2 * ROS_COV_DIM + 2] = INFINITE_VARIANCE; // default variance of z position, might get overwritten below if numAxes > 2

    for(int row = 0; row < numAxes; row++) {
       for(int col = 0; col < numAxes; col++) {
           pose.covariance[row * ROS_COV_DIM + col] = C(row, col); // copy position covariances
       }
    }

    pose.covariance[3 * ROS_COV_DIM + 3] = INFINITE_VARIANCE; // variance of x rotation
    pose.covariance[4 * ROS_COV_DIM + 4] = INFINITE_VARIANCE; // variance of y rotation
    pose.covariance[5 * ROS_COV_DIM + 5] = AVERAGE_ROTATION_VARIANCE; // variance of z rotation

    // Set twist (=velocities)
    twist.twist.linear.x = x(numAxes + 0);
    twist.twist.linear.y = x(numAxes + 1);
    twist.twist.linear.z = numAxes > 2 ? x(numAxes + 2) : 0.0;

    twist.covariance.fill(0.0);
    twist.covariance[2 * ROS_COV_DIM + 2] = INFINITE_VARIANCE; // default variance of z linear velocity, might get overwritten below if numAxes > 2

    for(int row = 0; row < numAxes; row++) {
       for(int col = 0; col < numAxes; col++) {
           twist.covariance[row * ROS_COV_DIM + col] = C(numAxes + row, numAxes + col); // copy velocity covariances
       }
    }

    twist.covariance[3 * ROS_COV_DIM + 3] = INFINITE_VARIANCE; // variance of x angular velocity
    twist.covariance[4 * ROS_COV_DIM + 4] = INFINITE_VARIANCE; // variance of y angular velocity
    twist.covariance[5 * ROS_COV_DIM + 5] = INFINITE_VARIANCE; // variance of z angular velocity
}


void GeometryUtils::poseToMeanAndCovariance(const geometry_msgs::PoseWithCovariance& pose, ObsVector& x, ObsMatrix& C, const Eigen::Affine3d& transformToApply)
{
    // ROS Pose is a 3D position + rotation relative to the message's coordinate frame
    Eigen::Affine3d poseInDetectionFrame;
    tf::poseMsgToEigen(pose.pose, poseInDetectionFrame);

    // ROS Covariance is a 6x6 matrix (xyz + xyz rotation) relative to the message's coordinate frame
    // We are not interested in pose rotation, so only take first 3 rows and columns
    Eigen::Matrix3d covInDetectionFrame;
    for(int row = 0; row < ROS_COV_DIM / 2; row++) {
        for(int col = 0; col < ROS_COV_DIM / 2; col++) {
            covInDetectionFrame(row, col) = pose.covariance[row * ROS_COV_DIM + col];
        }
    }

    // Transform pose and covariance into our reference frame, still in 3D
    Eigen::Affine3d poseInOurFrame = transformToApply * poseInDetectionFrame;
    Eigen::Vector3d positionInOurFrame = poseInOurFrame.translation();

    // For covariance, only the coordinate frame rotation is relevant (invariant w.r.t. translation)
    Eigen::Matrix3d detectionFrameToOurFrameRotation = transformToApply.linear().matrix();
    Eigen::Matrix3d covInOurFrame = detectionFrameToOurFrameRotation * covInDetectionFrame * detectionFrameToOurFrameRotation.transpose();

    // Convert from 3D to 2D by simply dropping the Z coordinate
    assert(OBS_DIM == 2);
    x = positionInOurFrame.head(OBS_DIM);
    C = covInOurFrame.topLeftCorner(OBS_DIM, OBS_DIM);
}


bool GeometryUtils::posePassesSanityCheck(const geometry_msgs::PoseWithCovariance& poseWithCovariance, bool checkOrientation)
{
    const geometry_msgs::Pose& pose = poseWithCovariance.pose;

    // Position
    const float MAX_REASONABLE_DISTANCE_FROM_ORIGIN = 1000.0f; // in meters
    float positionValues[] = { pose.position.x, pose.position.y, pose.position.z };
    for(size_t i = 0; i < sizeof(positionValues) / sizeof(positionValues[0]); i++) {
        if(!isfinite(positionValues[i]) || abs(positionValues[i]) > MAX_REASONABLE_DISTANCE_FROM_ORIGIN) {
            ROS_WARN("Suspicious coordinate value(s) in pose position encountered!");
            return false;
        }
    }

    // Orientation
    if(checkOrientation) {
        float squaredSum = 0;
        float orientationValues[] = { pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w };
        for(size_t i = 0; i < sizeof(orientationValues) / sizeof(orientationValues[0]); i++) {
            if(!isfinite(orientationValues[i])) {
                ROS_WARN("Non-finite pose orientation value(s) encountered!");
                return false;
            }
            squaredSum += orientationValues[i] * orientationValues[i];
        }

        if(abs(squaredSum - 1.0) > 0.05) {
            ROS_WARN("Pose orientation quaternion is not a unit quaternion!");
            return false;
        }
    }

    // Positive semi-definiteness of covariance matrix (x, y coordinates only)
    Eigen::Matrix2d cov;
    for(int row = 0; row < 2; row++)
        for(int col = 0; col < 2; col++)
            cov(row, col) = poseWithCovariance.covariance[row * ROS_COV_DIM + col];

    Eigen::Vector2cd eigenvals = cov.eigenvalues();
    bool positiveSemiDefinite = eigenvals(0).real() > 0 && eigenvals(1).real() > 0;
    if(!positiveSemiDefinite) {
        ROS_WARN_STREAM("Pose covariance matrix X,Y part is not positive semi-definite: " << std::endl << cov);
        return false;
    }

    return true;
}



} // end of namespace srl