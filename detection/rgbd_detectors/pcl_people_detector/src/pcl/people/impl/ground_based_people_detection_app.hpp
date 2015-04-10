/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
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
 * ground_based_people_detection_app.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_
#define PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_

#include <cmath>
#include <pcl/people/ground_based_people_detection_app.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

template <typename PointT>
pcl::people::GroundBasedPeopleDetectionApp<PointT>::GroundBasedPeopleDetectionApp ()
{
  // set default values for optional parameters:
  sampling_factor_ = 1;
  voxel_size_ = 0.06;
  groundplane_height_ = 2 * voxel_size_;
  optimize_groundplane_ = false;
  vertical_ = false;
  head_centroid_ = true;
  min_height_ = 1.3;
  max_height_ = 2.3;
  min_points_ = 30;     // this value is adapted to the voxel size in method "compute"
  max_points_ = 5000;   // this value is adapted to the voxel size in method "compute"
  dimension_limits_set_ = false;
  heads_minimum_distance_ = 0.3;
  show_rgb_image_ = false;

  // set flag values for mandatory parameters:
  sqrt_ground_coeffs_ = std::numeric_limits<float>::quiet_NaN();
  person_classifier_set_flag_ = false;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setInputCloud (PointCloudPtr& cloud)
{
  cloud_ = cloud;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setGround (Eigen::VectorXf& ground_coeffs)
{
  ground_coeffs_ = ground_coeffs;
  sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setGroundplaneHeight (float groundplane_height) {
  groundplane_height_ = groundplane_height;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setOptimizeGroundplane (bool optimize_groundplane) {
  optimize_groundplane_ = optimize_groundplane;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setSamplingFactor (int sampling_factor)
{
  sampling_factor_ = sampling_factor;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setVoxelSize (float voxel_size)
{
  voxel_size_ = voxel_size;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setIntrinsics (Eigen::Matrix3f intrinsics_matrix)
{
  intrinsics_matrix_ = intrinsics_matrix;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setDetectionFrameToRGBTransform (Eigen::Affine3f detection_frame_to_rgb_transform)
{
    detection_frame_to_rgb_transform_ = detection_frame_to_rgb_transform;
}


template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setClassifier (pcl::people::PersonClassifier<pcl::RGB> person_classifier)
{
  person_classifier_ = person_classifier;
  person_classifier_set_flag_ = true;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setSensorPortraitOrientation (bool vertical)
{
  vertical_ = vertical;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setHeightLimits (float min_height, float max_height)
{
  min_height_ = min_height;
  max_height_ = max_height;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setDimensionLimits (int min_points, int max_points)
{
  min_points_ = min_points;
  max_points_ = max_points;
  dimension_limits_set_ = true;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setMinimumDistanceBetweenHeads (float heads_minimum_distance)
{
  heads_minimum_distance_= heads_minimum_distance;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setHeadCentroid (bool head_centroid)
{
  head_centroid_ = head_centroid;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getHeightLimits (float& min_height, float& max_height)
{
  min_height = min_height_;
  max_height = max_height_;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getDimensionLimits (int& min_points, int& max_points)
{
  min_points = min_points_;
  max_points = max_points_;
}

template <typename PointT> float
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getMinimumDistanceBetweenHeads ()
{
  return (heads_minimum_distance_);
}

template <typename PointT> Eigen::VectorXf
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getGround ()
{
  if (sqrt_ground_coeffs_ != sqrt_ground_coeffs_)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::getGround] Floor parameters have not been set or they are not valid!\n");
  }
  return (ground_coeffs_);
}

template <typename PointT> bool
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getOptimizeGroundplane () {
  return optimize_groundplane_;
}

template <typename PointT> typename pcl::people::GroundBasedPeopleDetectionApp<PointT>::PointCloudPtr
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getNoGroundCloud ()
{
  return (no_ground_cloud_);
}

template <typename PointT> typename pcl::people::GroundBasedPeopleDetectionApp<PointT>::PointCloudPtr
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getGroundCloud ()
{
  return (ground_cloud_);
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setShowRgbImage(bool show)
{
  show_rgb_image_ = show;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::extractRGBFromPointCloud (PointCloudPtr input_cloud, cv::Mat& rgb_image_cv)
{
  // This allocation will only happen once as long as the dimensions don't change
  rgb_image_cv.create(input_cloud->height, input_cloud->width, CV_32FC3);

  // Extract RGB information from a point cloud and output the corresponding RGB image
  for (int x = 0; x < input_cloud->width; x++)
  {
    for (int y = 0; y < input_cloud->height; y++)
    { 
      rgb_image_cv.at<cv::Vec3f>(y, x)[0] = (*input_cloud)(x, y).b / 255.0f; // OpenCV uses BGR format
      rgb_image_cv.at<cv::Vec3f>(y, x)[1] = (*input_cloud)(x, y).g / 255.0f;
      rgb_image_cv.at<cv::Vec3f>(y, x)[2] = (*input_cloud)(x, y).r / 255.0f;
    }
  }
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::rotateRgbImage(Eigen::Rotation2Df& resultingRotationMatrix)
{
    // Determine angle by which we need to rotate the 2D RGB image to obtain upright, axis-parallel bounding boxes
    // FIXME: This can be precomputed
    Eigen::Vector3f actualUpVectorInDetectionFrame, actualUpVectorInCameraFrame3d;
    Eigen::Vector2f actualUpVectorInCameraFrame, desiredUpVectorInCameraFrame;
    actualUpVectorInDetectionFrame << 0.0f, 1.0f, 0.0f;

    actualUpVectorInCameraFrame3d = intrinsics_matrix_ * detection_frame_to_rgb_transform_ * actualUpVectorInDetectionFrame;
    actualUpVectorInCameraFrame3d /= actualUpVectorInCameraFrame3d(2);

    actualUpVectorInCameraFrame = actualUpVectorInCameraFrame3d.head(2).normalized();
    desiredUpVectorInCameraFrame << 0, -1;

    float rotationAngle = std::atan2(actualUpVectorInCameraFrame.y(), actualUpVectorInCameraFrame.x()) - atan2(desiredUpVectorInCameraFrame.y(), desiredUpVectorInCameraFrame.x());
    rotationAngle *= 180.0f / M_PI;

    int multipleOf90RotationAngle = ((int)round(rotationAngle / 90.0f) % 4) * 90;
    PCL_DEBUG("Required rotation angle is %d° (%f°)\n", multipleOf90RotationAngle, rotationAngle);

    // Create corresponding rotation matrix with which we can rotate the detected clusters from the depth image
    resultingRotationMatrix = Eigen::Rotation2Df(multipleOf90RotationAngle / 180.0f * M_PI);

    // Rotate RGB image if necessary. We only support multiples of 90 degrees to be faster
    // 0 : flip vertical; 1 flip horizontal. From http://stackoverflow.com/a/16278334/554355
    bool const flip_horizontal_or_vertical = multipleOf90RotationAngle > 0 ? 1 : 0;
    int const number = std::abs(multipleOf90RotationAngle / 90);
    for(int i = 0; i != number; ++i){
       cv::transpose(rgb_image_cv_, rgb_image_cv_);
       cv::flip(rgb_image_cv_, rgb_image_cv_, flip_horizontal_or_vertical);
    }
}

template <typename PointT> bool
pcl::people::GroundBasedPeopleDetectionApp<PointT>::compute (std::vector<pcl::people::PersonCluster<PointT> >& clusters)
{
  // Check if all mandatory variables have been set:
  if (sqrt_ground_coeffs_ != sqrt_ground_coeffs_)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::compute] Floor parameters have not been set or they are not valid!\n");
    return (false);
  }
  if (cloud_ == NULL)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::compute] Input cloud has not been set!\n");
    return (false);
  }
  if (intrinsics_matrix_(0) == 0)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::compute] Camera intrinsic parameters have not been set!\n");
    return (false);
  }
  if (!person_classifier_set_flag_)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::compute] Person classifier has not been set!\n");
    return (false);
  }

  if (!dimension_limits_set_)    // if dimension limits have not been set by the user
  {
    // Adapt thresholds for clusters points number to the voxel size:
    max_points_ = int(float(max_points_) * std::pow(0.06/voxel_size_, 2));
    if (voxel_size_ > 0.06)
      min_points_ = int(float(min_points_) * std::pow(0.06/voxel_size_, 2));
  }

  // Extract RGB image
  extractRGBFromPointCloud(cloud_, rgb_image_cv_);

  // Downsample of sampling_factor in every dimension:
  if (sampling_factor_ != 1)
  {
    PointCloudPtr cloud_downsampled(new PointCloud);
    cloud_downsampled->width = (cloud_->width)/sampling_factor_;
    cloud_downsampled->height = (cloud_->height)/sampling_factor_;
    cloud_downsampled->points.resize(cloud_downsampled->height*cloud_downsampled->width);
    cloud_downsampled->is_dense = cloud_->is_dense;
    for (int j = 0; j < cloud_downsampled->width; j++)
    {
      for (int i = 0; i < cloud_downsampled->height; i++)
      {
        (*cloud_downsampled)(j,i) = (*cloud_)(sampling_factor_*j,sampling_factor_*i);
      }
    }
    (*cloud_) = (*cloud_downsampled);
  }

  // Voxel grid filtering:
  PointCloudPtr cloud_filtered(new PointCloud);
  pcl::VoxelGrid<PointT> voxel_grid_filter_object;
  voxel_grid_filter_object.setInputCloud(cloud_);
  voxel_grid_filter_object.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
  voxel_grid_filter_object.filter (*cloud_filtered);

  // Ground plane removal and update:
  pcl::IndicesPtr inliers(new std::vector<int>);
  boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_filtered));
  ground_model->selectWithinDistance(ground_coeffs_, 0.5 * groundplane_height_, *inliers);
  no_ground_cloud_ = PointCloudPtr (new PointCloud);
  ground_cloud_ = PointCloudPtr (new PointCloud);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud_);

  // Optimize groundplane, if requested
  if(optimize_groundplane_) {
      if (inliers->size () >= (300 * 0.06 / voxel_size_ / std::pow (static_cast<double> (sampling_factor_), 2)))
        ground_model->optimizeModelCoefficients (*inliers, ground_coeffs_, ground_coeffs_);
      else
        PCL_INFO ("No groundplane update, because number of matching points is too low!\n");
  }

  // For debugging: Output extracted ground plane points
  extract.setNegative(false);
  extract.filter(*ground_cloud_);

  // Euclidean clustering:
  std::vector<pcl::PointIndices> cluster_indices;
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(no_ground_cloud_);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(2 * 0.06);
  ec.setMinClusterSize(min_points_);
  ec.setMaxClusterSize(max_points_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(no_ground_cloud_);
  ec.extract(cluster_indices);

  // Head based sub-clustering
  pcl::people::HeadBasedSubclustering<PointT> subclustering;
  subclustering.setInputCloud(no_ground_cloud_);
  subclustering.setGround(ground_coeffs_);
  subclustering.setInitialClusters(cluster_indices);
  subclustering.setHeightLimits(min_height_, max_height_);
  subclustering.setDimensionLimits(min_points_, max_points_);
  subclustering.setMinimumDistanceBetweenHeads(heads_minimum_distance_);
  subclustering.setSensorPortraitOrientation(vertical_);
  subclustering.subcluster(clusters);

  // Properly orient the RGB image (e.g. in case of vertical setup)
  Eigen::Vector2f oldImageCenter(rgb_image_cv_.cols / 2.0f, rgb_image_cv_.rows / 2.0f);
  Eigen::Rotation2Df resultingRotationMatrix(0.0f);
  rotateRgbImage(resultingRotationMatrix);
  Eigen::Vector2f newImageCenter(rgb_image_cv_.cols / 2.0f, rgb_image_cv_.rows / 2.0f);

  // RGB image visualization
  if(show_rgb_image_) {
    rgb_image_cv_visualization_.create(rgb_image_cv_.size(), rgb_image_cv_.type());
    rgb_image_cv_.copyTo(rgb_image_cv_visualization_);
  }

  // Person confidence evaluation with HOG+SVM:
  for(typename std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    //Evaluate confidence for the current PersonCluster:
    Eigen::Vector3f tcenter = it->getTCenter();
    Eigen::Vector3f centroid = intrinsics_matrix_ * detection_frame_to_rgb_transform_ * tcenter; // this transforms the coordinates back into RGB optical frame
    centroid /= centroid(2);
    Eigen::Vector2f centroid2d = resultingRotationMatrix * (centroid.head(2) - oldImageCenter) + newImageCenter; // take rotation of RGB image into account

    Eigen::Vector3f ttop = it->getTTop();
    Eigen::Vector3f top = intrinsics_matrix_ * detection_frame_to_rgb_transform_ * ttop; // this transforms the coordinates back into RGB optical frame
    top /= top(2);
    Eigen::Vector2f top2d = resultingRotationMatrix * (top.head(2) - oldImageCenter) + newImageCenter;

    Eigen::Vector3f tbottom = it->getTBottom();
    Eigen::Vector3f bottom = intrinsics_matrix_ * detection_frame_to_rgb_transform_ * tbottom; // this transforms the coordinates back into RGB optical frame
    bottom /= bottom(2);
    Eigen::Vector2f bottom2d = resultingRotationMatrix * (bottom.head(2) - oldImageCenter) + newImageCenter;

    // NOTE: If this crashes, the transform from depth points back into the RGB frame (using the camera
    // intrinsics matrix) is probably wrong. Also check the RGB image rotation property of the classifier.
    cv::Rect personRect;
    it->setPersonConfidence(person_classifier_.evaluate(rgb_image_cv_, bottom2d, top2d, centroid2d, rgb_image_cv_visualization_, personRect));
    it->setImageRegion(personRect);
  }

  if(show_rgb_image_) {
      cv::imshow("RGB image visualization", rgb_image_cv_visualization_);
      cv::waitKey(1);
  }
 
  return (true);
}

template <typename PointT>
pcl::people::GroundBasedPeopleDetectionApp<PointT>::~GroundBasedPeopleDetectionApp ()
{
  // TODO Auto-generated destructor stub
}
#endif /* PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_ */
