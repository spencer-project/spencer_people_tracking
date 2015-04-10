/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 * ROS wrapper copyright (c) 2013-2015 Timm Linder, Social Robotics Laboratory, University of Freiburg
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */
  
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <vector>
#include <set>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_vision_msgs/PersonROIs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


#include "util/marker_utils.h"
#include "util/person_cluster_visualizer.h"

using namespace std;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

pcl::people::GroundBasedPeopleDetectionApp<PointType> g_people_detector; // People detection object
PointCloudType::Ptr g_cloud; // Our point-cloud (global to prevent repeated allocs)

// Global parameters
int g_frameSkip;
int g_detectionIdOffset, g_detectionIdIncrement;
double g_minConfidence;
string g_detectionFrame, g_baseLinkFrame, g_imageSourceRGB, g_imageSourceDepth, g_modalityName;
Eigen::Matrix3d g_intrinsicsMatrix = Eigen::Matrix3d::Zero();
boost::mutex g_intrinsicsMutex;

// ROS components
boost::shared_ptr<ros::NodeHandle> g_paramNodeHandle;
boost::shared_ptr<tf::TransformListener> g_transformListener;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
ros::Publisher g_groundCloudPublisher, g_noGroundCloudPublisher, g_markerPublisher, g_detectedPersonsPublisher, g_personROIsPublisher;

set<unsigned int> g_oldDetectionIds;
unsigned int g_currentDetectionId;


Eigen::Affine3d getCameraTransform(string frameId) {
    // Get the current position of the camera with regard to base_link
    tf::StampedTransform opticalFrameToBaseLinkTransform;
    g_transformListener->lookupTransform(g_baseLinkFrame, frameId, ros::Time(0), opticalFrameToBaseLinkTransform);

    Eigen::Affine3d opticalFrameToBaseLinkTransformEigen;
    tf::transformTFToEigen(opticalFrameToBaseLinkTransform, opticalFrameToBaseLinkTransformEigen);

    return opticalFrameToBaseLinkTransformEigen;
}

Eigen::Hyperplane<float, 3> getGroundPlane()
{
    // Get ground plane estimate [normal_x normal_y normal_z d] from parameters
    std::vector<float> original_ground_coeffs;
    string original_ground_coeffs_string;
    g_paramNodeHandle->getParam("ground_coeffs", original_ground_coeffs_string); // FIXME: Replace by dynparam
    {
        std::istringstream is(original_ground_coeffs_string);
        double value;
        while( is >> value ) {
            original_ground_coeffs.push_back(value);
        }
    }

    if (original_ground_coeffs.size() < 4) {
        ROS_WARN("Ground plane parameters not set, using defaults!");
        original_ground_coeffs.clear();
        original_ground_coeffs.push_back(0);
        original_ground_coeffs.push_back(0);
        original_ground_coeffs.push_back(1);
        original_ground_coeffs.push_back(0);
    }

    return Eigen::Hyperplane<float, 3>(
            Eigen::Vector3f(original_ground_coeffs[0], original_ground_coeffs[1], original_ground_coeffs[2]),
            original_ground_coeffs[3]);
}

Eigen::Quaterniond getGroundPlaneRotation(Eigen::Hyperplane<float, 3> groundPlane) {
    Eigen::Quaternion<double> groundPlaneRotation;
    groundPlaneRotation.setFromTwoVectors(Eigen::Vector3d(0,1,0), groundPlane.normal().cast<double>());
    return groundPlaneRotation;
}

Eigen::Affine3d getGroundPlaneTransform(Eigen::Hyperplane<float, 3> groundPlane) {
    return Eigen::Translation3d(groundPlane.normal().cast<double>() * groundPlane.offset())
            * getGroundPlaneRotation(groundPlane);
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::Ptr& msg) {
    boost::array<double, 9> intrinsics = msg->K;
    Eigen::Matrix3d intrinsicsMatrix;
    for(int row = 0; row < 3; row++) for(int col = 0; col < 3; col++) intrinsicsMatrix(row, col) = intrinsics[row * 3 + col];

    boost::lock_guard<boost::mutex> lock(g_intrinsicsMutex);
    if(intrinsicsMatrix !=  g_intrinsicsMatrix) {
        g_intrinsicsMatrix = intrinsicsMatrix;
    }
}

float mapConfidenceToProbability(float personConfidence) {
    // Manually-fit logistic (sigmoid) function FIXME: Tune parameters
    float scaledPersonConfidence = (personConfidence - g_minConfidence) / (1.5f - g_minConfidence);
    float rescaledPersonConfidence = scaledPersonConfidence * 2.0f; // map onto [0 +2] range
    float denominator = 1 + exp(-rescaledPersonConfidence);
    return 1.0f / denominator;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::Ptr& msg)
{
    // Process only every g_frameSkip+1-th frame
    static int frameSkipCounter = 0;
    if(frameSkipCounter < g_frameSkip) {
        frameSkipCounter++;
        return;
    }
    else frameSkipCounter = 0;

    // Get camera transform
    Eigen::Affine3d cameraTransform;

    string cameraFrame = msg->header.frame_id;
    try {
        cameraTransform = getCameraTransform(cameraFrame);
    }
    catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(5.0, "Transform from %s to %s unavailable: %s", cameraFrame.c_str(), g_baseLinkFrame.c_str(), ex.what());
        return;
    }

    // Use current timestamp for marker messages
    MarkerUtils::setTimeOfLastMessage(msg->header.stamp);

    // Set up transform from base_link into our own coordinate frame
    // (Y down, X right, Z into screen)
    // For the detector to work, Y MUST point downwards!
    Eigen::Affine3d baseLinkToDetectionFrame = Eigen::Affine3d(
            Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ())
        );

    // Publish inverse transform on TF
    tf::Transform detectionFrameToBaseLinkTF; tf::transformEigenToTF(baseLinkToDetectionFrame.inverse(), detectionFrameToBaseLinkTF);
    g_transformBroadcaster->sendTransform(tf::StampedTransform(detectionFrameToBaseLinkTF, msg->header.stamp, g_baseLinkFrame, g_detectionFrame));

    // Get ground plane from parameters
    Eigen::Hyperplane<float, 3> groundPlane = getGroundPlane();

    // Transform groundplane into detection frame
    Eigen::Hyperplane<float, 3> groundPlaneInDetectionFrame = groundPlane.transform(baseLinkToDetectionFrame.cast<float>());

    // Set the people detector ground plane
    Eigen::VectorXf groundPlaneCoeffs(4);
    groundPlaneCoeffs.head(3) = groundPlaneInDetectionFrame.normal();
    groundPlaneCoeffs.tail(1)[0] = -groundPlaneInDetectionFrame.offset(); // not sure why minus sign is needed
    g_people_detector.setGround(groundPlaneCoeffs);

    // Convert cloud into PCL format
    pcl::fromROSMsg(*msg, *g_cloud);

    // cameraTransform is from optical frame to base_link
    // baseLinkToDetectionFrame is from base_link into an y-pointing-down reference frame
    Eigen::Affine3d cameraToDetectionFrame = baseLinkToDetectionFrame * cameraTransform;

    // Rotate pointcloud so it is upright
    // FIXME: This might be very slow. However, just rotating the ground plane
    // does interfere with some implementation details of the people detection algorithm.
    pcl::transformPointCloud(*g_cloud, *g_cloud, cameraToDetectionFrame);
    g_cloud->header.frame_id = g_detectionFrame; // Update frame, since we are applying a transform (important for visualization)

    // Set transform to RGB frame
    g_people_detector.setDetectionFrameToRGBTransform(cameraToDetectionFrame.inverse().cast<float>());

    // Set RGB camera intrinsics matrix from camera info topic
    {
        boost::lock_guard<boost::mutex> lock(g_intrinsicsMutex);
        if(g_intrinsicsMatrix == Eigen::Matrix3d::Zero()) {
            ROS_WARN_THROTTLE(5.0, "Intrinsics matrix not set, make sure that the camera_info topic is set correctly and being published!");
            return;
        }
        else {
            g_people_detector.setIntrinsics(g_intrinsicsMatrix.cast<float>());
        }
    }

    // Perform people detection on the new cloud
    typedef std::vector<pcl::people::PersonCluster<PointType> > Clusters;
    Clusters clusters;   // vector containing persons clusters
    g_people_detector.setInputCloud(g_cloud);
    g_people_detector.compute(clusters);                              // perform people detection

    // Apply minimum confidence level constraint
    Clusters clustersWithinConfidenceBounds;

    for (Clusters::iterator it = clusters.begin(); it != clusters.end(); ++it) {
        double confidence = it->getPersonConfidence();
        if (confidence > g_minConfidence)             // draw only people with confidence above a threshold
        {
            clustersWithinConfidenceBounds.push_back(*it);
        }
    }

    // Merge closeby persons (this should already be handled by the clustering, but somehow sometimes fails...)
    Clusters finalFilteredClusters;
    const double MIN_DISTANCE = 0.30;
    for(Clusters::iterator it = clustersWithinConfidenceBounds.begin(); it != clustersWithinConfidenceBounds.end(); ++it) {
        Eigen::Vector3f bottom = it->getTBottom();
        bottom.y() = 0; // ignore height
        bool overlapsWithOtherCluster = false;

        for(Clusters::iterator it2 = finalFilteredClusters.begin(); it2 != finalFilteredClusters.end(); ++it2) {
            Eigen::Vector3f otherBottom = it2->getTBottom();
            otherBottom.y() = 0; // ignore height

            if((bottom - otherBottom).norm() <= MIN_DISTANCE) {
                overlapsWithOtherCluster = true;
                break;
            }
        }

        if(!overlapsWithOtherCluster) finalFilteredClusters.push_back(*it);
    }

    // Get updated floor coefficients (after model optimization)
    Eigen::VectorXf updated_ground_coeffs = g_people_detector.getGround();
    Eigen::Hyperplane<float, 3> updatedGroundPlaneInDetectionFrame(
            Eigen::Vector3f(updated_ground_coeffs[0], updated_ground_coeffs[1], updated_ground_coeffs[2]), -updated_ground_coeffs[3]);

    //
    // Publish detections on ROS
    //
    Eigen::Quaterniond upVectorRotationInDetectionFrame;
    upVectorRotationInDetectionFrame.setFromTwoVectors(Eigen::Vector3d(0,0,1), groundPlaneInDetectionFrame.normal().cast<double>());
    upVectorRotationInDetectionFrame = upVectorRotationInDetectionFrame * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d(0,0,1));

    map<Clusters::value_type*, int> detectionIdMap;
    static int currentDetectionId = g_detectionIdOffset;

    if(g_detectedPersonsPublisher.getNumSubscribers()) {
        static int numDetectedPersonsMessages = 0;

        spencer_tracking_msgs::DetectedPersons detectedPersons;
        detectedPersons.header.stamp = msg->header.stamp;
        detectedPersons.header.frame_id = g_detectionFrame;
        detectedPersons.header.seq = numDetectedPersonsMessages++;

        for (Clusters::iterator it = finalFilteredClusters.begin(); it != finalFilteredClusters.end(); ++it)
        {
            spencer_tracking_msgs::DetectedPerson detectedPerson;

            // Set observation ID and confidence
            detectedPerson.detection_id = currentDetectionId;
            detectedPerson.confidence = mapConfidenceToProbability(it->getPersonConfidence());
            detectedPerson.modality = g_modalityName;
            detectionIdMap[&*it] = detectedPerson.detection_id;

            // Set pose
            const double AVERAGE_POSITION_VARIANCE = pow(0.17 /* stddev in meters */, 2);
            const double INFINITE_VARIANCE = std::numeric_limits<double>::infinity();

            detectedPerson.pose.covariance.fill(0.0);
            detectedPerson.pose.covariance[0 * 6 + 0] = AVERAGE_POSITION_VARIANCE; // x
            detectedPerson.pose.covariance[1 * 6 + 1] = AVERAGE_POSITION_VARIANCE; // y
            detectedPerson.pose.covariance[2 * 6 + 2] = AVERAGE_POSITION_VARIANCE; // z
            detectedPerson.pose.covariance[3 * 6 + 3] = INFINITE_VARIANCE; // x rotation
            detectedPerson.pose.covariance[4 * 6 + 4] = INFINITE_VARIANCE; // y rotation
            detectedPerson.pose.covariance[5 * 6 + 5] = INFINITE_VARIANCE; // z rotation

            Eigen::Vector3f center = it->getTCenter();
            tf::poseEigenToMsg(Eigen::Translation3d(center.cast<double>()) * Eigen::Affine3d(upVectorRotationInDetectionFrame), detectedPerson.pose.pose);

            // We're done with this observation
            detectedPersons.detections.push_back(detectedPerson);
            currentDetectionId += g_detectionIdIncrement;
        }

        // Finally publish all detections
        g_detectedPersonsPublisher.publish(detectedPersons);
    }

    // Publish detection rectangles (regions of interest)
    if(g_personROIsPublisher.getNumSubscribers() > 0) {
        spencer_vision_msgs::PersonROIs personROIs;

        static int imageRegionCounter = 0;
        personROIs.header.frame_id = msg->header.frame_id;
        personROIs.header.stamp = msg->header.stamp;
        personROIs.header.seq = imageRegionCounter++;

        personROIs.rgb_topic = g_imageSourceRGB;
        personROIs.depth_topic = g_imageSourceDepth;

        for (Clusters::iterator it = finalFilteredClusters.begin(); it != finalFilteredClusters.end(); ++it)
        {
            const cv::Rect imageRegion = it->getImageRegion();

            spencer_vision_msgs::PersonROI personROI;
            personROI.detection_id = detectionIdMap.find(&*it) != detectionIdMap.end() ? detectionIdMap[&*it] : currentDetectionId += g_detectionIdIncrement;
            personROI.roi.x_offset = imageRegion.x;
            personROI.roi.y_offset = imageRegion.y;
            personROI.roi.width = imageRegion.width;
            personROI.roi.height = imageRegion.height;

            personROIs.elements.push_back(personROI);
        }

        g_personROIsPublisher.publish(personROIs);
    }

    //
    // Visualization
    //

    // All voxels above ground
    if(g_noGroundCloudPublisher.getNumSubscribers()) {
        PointCloudType::Ptr noGroundCloud = g_people_detector.getNoGroundCloud();
        sensor_msgs::PointCloud2 noGroundCloudROS;
        pcl::toROSMsg(*noGroundCloud, noGroundCloudROS);
        g_noGroundCloudPublisher.publish(noGroundCloudROS);
    }

    // All voxels in the ground plane
    if(g_groundCloudPublisher.getNumSubscribers()) {
        PointCloudType::Ptr groundCloud = g_people_detector.getGroundCloud();
        sensor_msgs::PointCloud2 groundCloudROS;
        pcl::toROSMsg(*groundCloud, groundCloudROS);
        g_groundCloudPublisher.publish(groundCloudROS);
    }

    // Publish miscellaneous markers if there are subscribers
    if(g_markerPublisher.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray markerArray;

        // Original ground plane
        visualization_msgs::Marker groundPlaneMarker = MarkerUtils::createMarker("Groundplane", g_detectionFrame);
        groundPlaneMarker.type = visualization_msgs::Marker::CUBE;
        groundPlaneMarker.color.r = 0.5;
        groundPlaneMarker.color.g = 0.5;
        groundPlaneMarker.color.b = 0.5;
        groundPlaneMarker.color.a = 0.5;
        groundPlaneMarker.scale.x = 20.0;
        groundPlaneMarker.scale.y = 0.06; //g_people_detector.voxel_size_;
        groundPlaneMarker.scale.z = 20.0;
        Eigen::Translation3d shiftForward = Eigen::Translation3d(0,0,0.5*groundPlaneMarker.scale.z); // move plane in front of camera
        tf::poseEigenToMsg(shiftForward * getGroundPlaneTransform(groundPlaneInDetectionFrame), groundPlaneMarker.pose);
        markerArray.markers.push_back(groundPlaneMarker);

        if(g_people_detector.getOptimizeGroundplane()) {
            // Updated estimate of the ground plane, after model optimization
            visualization_msgs::Marker updatedGroundPlaneMarker = groundPlaneMarker;
            updatedGroundPlaneMarker.ns = "Groundplane_Optimized";
            updatedGroundPlaneMarker.color.g = 0;
            tf::poseEigenToMsg(shiftForward * getGroundPlaneTransform(updatedGroundPlaneInDetectionFrame), updatedGroundPlaneMarker.pose);
            markerArray.markers.push_back(updatedGroundPlaneMarker);
        }

        // Publish all markers
        g_markerPublisher.publish(markerArray);
    }

    // Visualize detections
    PersonClusterVisualizer<PointType>::visualize("FinalBeforeHOGClassify", clusters);
    PersonClusterVisualizer<PointType>::visualize("FinalAfterHOGClassify", clustersWithinConfidenceBounds);
    PersonClusterVisualizer<PointType>::endFrame();
}

int main (int argc, char** argv)
{
  // Set up ROS node
  ros::init(argc, argv, "pcl_people_detector");
  ros::NodeHandle nodeHandle("");
  g_paramNodeHandle = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

  // Parameters
  string svm_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/parameters/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  g_minConfidence = -1.6;
  double min_height = 1.2;
  double max_height = 2.0;
  double voxel_size = 0.06, groundplane_height = 0.4;
  int queueSize = 1;
  g_frameSkip = 0;
  g_baseLinkFrame = "base_link";
  g_detectionFrame = "pcl_people_detector_frame";
  string input_cloud_topic = "/camera/depth_registered/points";
  string camera_info_topic = "/camera/rgb/camera_info";
  int rgb_image_rotation_deg = 0;
  bool optimize_groundplane = false;
  bool rgb_file_export = false;
  bool rgb_visualization = false;
  bool gpu_classifier = true;
  g_detectionIdOffset = 0;
  g_detectionIdIncrement = 1;
  g_modalityName = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_RGBD;

  g_paramNodeHandle->getParam("svm_file", svm_filename);
  g_paramNodeHandle->getParam("min_confidence", g_minConfidence);
  g_paramNodeHandle->getParam("min_person_height", min_height);
  g_paramNodeHandle->getParam("max_person_height", max_height);
  g_paramNodeHandle->getParam("voxel_size", voxel_size);
  g_paramNodeHandle->getParam("groundplane_height", groundplane_height);
  g_paramNodeHandle->getParam("queue_size", queueSize);
  g_paramNodeHandle->getParam("frame_skip", g_frameSkip);
  g_paramNodeHandle->getParam("base_link_frame", g_baseLinkFrame);
  g_paramNodeHandle->getParam("detection_frame", g_detectionFrame);
  g_paramNodeHandle->getParam("input_topic", input_cloud_topic);
  g_paramNodeHandle->getParam("camera_info_topic", camera_info_topic);
  g_paramNodeHandle->getParam("optimize_groundplane", optimize_groundplane);
  g_paramNodeHandle->getParam("rgb_file_export", rgb_file_export);
  g_paramNodeHandle->getParam("rgb_visualization", rgb_visualization);
  g_paramNodeHandle->getParam("gpu_classifier", gpu_classifier); // requires OpenCV to be compiled with WITH_CUDA flag
  g_paramNodeHandle->getParam("detection_id_offset", g_detectionIdOffset);
  g_paramNodeHandle->getParam("detection_id_increment", g_detectionIdIncrement);
  g_paramNodeHandle->getParam("modality_name", g_modalityName);

  ROS_INFO_STREAM("Loading learned SVM classifier from file " << svm_filename);

  // Check if OpenCV was compiled with CUDA support
  if(gpu_classifier && 0 == cv::gpu::getCudaEnabledDeviceCount()) {
    gpu_classifier = false;
    ROS_ERROR_STREAM("OpenCV was compiled without GPU (CUDA) support. Overriding gpu_classifier parameter and setting it to False.");
  }
  ROS_INFO_STREAM("GPU acceleration is " << (gpu_classifier ? "enabled" : "disabled"));

  
  // Image source is used to publish ROIs. Our ROIs are based upon a rotated image (due to sensor orientation),
  // so we cannot use the original RGB image as source
  // We assume that a separate, stand-alone node will publish the rotated image on this topic.
  const string imageSourceSuffix = "rotated_upright";
  string imageSourceBase = ros::names::remap(input_cloud_topic);
  imageSourceBase = imageSourceBase.substr(0, imageSourceBase.find_last_of("/"));
  imageSourceBase = imageSourceBase.substr(0, imageSourceBase.find_last_of("/"));
  g_imageSourceRGB   = imageSourceBase + "/rgb/" + imageSourceSuffix + "/image_rect_color";
  g_imageSourceDepth = imageSourceBase + "/depth_registered/" + imageSourceSuffix + "/image_raw";

  // Set up TF listener & broadcaster
  g_transformListener = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(nodeHandle, ros::Duration(1.0)));
  g_transformBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

  //
  // Set up ROS publishers
  //

  // In private namespace (mostly used for debugging)
  g_groundCloudPublisher = g_paramNodeHandle->advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
  g_noGroundCloudPublisher = g_paramNodeHandle->advertise<sensor_msgs::PointCloud2>("no_ground_cloud", 1);
  g_markerPublisher = g_paramNodeHandle->advertise<visualization_msgs::MarkerArray>("misc_marker_array", 5);

  // Absolute topic paths (outputs used for further processing)
  g_detectedPersonsPublisher = nodeHandle.advertise<spencer_tracking_msgs::DetectedPersons>("/spencer/perception/detected_persons", 10);
  g_personROIsPublisher = g_paramNodeHandle->advertise<spencer_vision_msgs::PersonROIs>("rois", 10);

  PersonClusterVisualizer<PointType>::createPublisher(*g_paramNodeHandle, g_detectionFrame);

  // Create point cloud buffer
  g_cloud = PointCloudType::Ptr(new PointCloudType);

  // Subscribe to point cloud and camera info messages
  ros::Subscriber pointCloudSubscriber = nodeHandle.subscribe(input_cloud_topic, queueSize, &pointCloudCallback);
  ros::Subscriber cameraInfoSubscriber = nodeHandle.subscribe(camera_info_topic, 1, &cameraInfoCallback);

  // Create classifier for people detection:  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  if(gpu_classifier) {
      person_classifier.useOpenCVGpuClassifier(); // will automatically load default weights provided by OpenCV
  }
  else {
      person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM
  }
  person_classifier.enableRGBFileExport(rgb_file_export);  // set to true to export PNGs to working directory

  // People detection app initialization:
  g_people_detector.setGroundplaneHeight(groundplane_height);        // set ground plane height
  g_people_detector.setOptimizeGroundplane(optimize_groundplane);    // enable optimization of ground plane?
  g_people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  g_people_detector.setClassifier(person_classifier);                // set person classifier
  g_people_detector.setHeightLimits(min_height, max_height);         // set height limits
  g_people_detector.setShowRgbImage(rgb_visualization);              // set RGB image visualization
  //g_people_detector.setSensorPortraitOrientation(true);            // set sensor orientation to vertical


  // Main loop
  ROS_INFO_STREAM("Now listening for input point clouds at " << input_cloud_topic << " and camera info at " << camera_info_topic << " ...");
  ros::spin();

  return 0;
}
