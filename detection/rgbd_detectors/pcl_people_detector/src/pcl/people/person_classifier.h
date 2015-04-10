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
 * person_classifier.h
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */
 
#ifndef PCL_PEOPLE_PERSON_CLASSIFIER_H_
#define PCL_PEOPLE_PERSON_CLASSIFIER_H_

#include <pcl/people/person_cluster.h>
#include <pcl/people/hog.h>

#include <opencv2/opencv.hpp>

namespace pcl
{
  namespace people
  {
    template <typename PointT> class PersonClassifier;

    template <typename PointT>
    class PersonClassifier
    {
    protected:

      /** \brief Height of the image patch to classify. */
      int window_height_;          
      
      /** \brief Width of the image patch to classify. */
      int window_width_;          

      /** \brief SVM offset. */
      float SVM_offset_;
      
      /** \brief SVM weights vector. */
      std::vector<float> SVM_weights_;  

      /** \brief Enable export of RGB images as PNG files into current working directory. */
      bool enable_rgb_file_export_;

      /** \brief Use GPU-accelerated classifier? (possibly requires different weights than normal classifier) */
      bool use_opencv_gpu_classifier_;

    public:

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef boost::shared_ptr<PointCloud> PointCloudPtr;

      /** \brief Constructor. */
      PersonClassifier ();

      /** \brief Destructor. */
      virtual ~PersonClassifier ();

      /** \brief Load SVM parameters from a text file. 
       *
       * \param[in] svm_filename Filename containing SVM parameters.
       * 
       * \return true if SVM has been correctly set, false otherwise.
       */
      bool
      loadSVMFromFile (std::string svm_filename);

      /**
       * \brief Set trained SVM for person confidence estimation.
       * 
       * \param[in] window_height Detection window height.
       * \param[in] window_width Detection window width.
       * \param[in] SVM_weights SVM weights vector.
       * \param[in] SVM_offset SVM offset.
       */
      void
      setSVM (int window_height, int window_width, std::vector<float> SVM_weights, float SVM_offset);

      /**
       * \brief Get trained SVM for person confidence estimation.
       * 
       * \param[out] window_height Detection window height.
       * \param[out] window_width Detection window width.
       * \param[out] SVM_weights SVM weights vector.
       * \param[out] SVM_offset SVM offset.
       */
      void
      getSVM (int& window_height, int& window_width, std::vector<float>& SVM_weights, float& SVM_offset);

      /**
       * \brief Enable export of person RGB images as PNG files into current working directory.
       *
       * \param[in] enable_rgb_file_export Set to true to enable export.
       */
      void
      enableRGBFileExport (bool enable_rgb_file_export);

      /**
       * \brief Compute person confidence for a given PersonCluster.
       * 
       * \param[in] rgbImage The input image (pointer to a point cloud containing RGB information).
       * \param[in] bottom Theoretical bottom point of the cluster projected to the image.
       * \param[in] top Theoretical top point of the cluster projected to the image.
       * \param[in] centroid Theoretical centroid point of the cluster projected to the image.
       * \param[in] verticalRGB If true, the RGB sensor is considered to be vertically placed (portrait mode).
       * \param[in, out] rgbImageVisualization RGB image to draw detection rectangles onto.
       * \param[out] Detection rectangle in the RGB image
       * \return The person confidence.
       */
      double
      evaluate (cv::Mat& image, Eigen::Vector2f& bottom, Eigen::Vector2f& top, Eigen::Vector2f& centroid, cv::Mat& rgbImageVisualization, cv::Rect& personRect);

      /**
       * \brief Use a GPU-accelerated HOG detector provided by OpenCV, as opposed to the original CPU-based manual implementation.
       * Calling this function will initialize the linear SVM weights with cv::gpu::HOGDescriptor::getPeopleDetector64x128() (and an offset of 0).
       * The weights can be overridden using setSVM().
       */
      void
      useOpenCVGpuClassifier(bool enable = true);
    };
  } /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/person_classifier.hpp>
#endif /* PCL_PEOPLE_PERSON_CLASSIFIER_H_ */
