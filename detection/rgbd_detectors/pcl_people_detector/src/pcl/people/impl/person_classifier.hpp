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
 * person_classifier.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#include <pcl/people/person_classifier.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <assert.h>


#ifndef PCL_PEOPLE_PERSON_CLASSIFIER_HPP_
#define PCL_PEOPLE_PERSON_CLASSIFIER_HPP_

template <typename PointT>
pcl::people::PersonClassifier<PointT>::PersonClassifier () : enable_rgb_file_export_(false), use_opencv_gpu_classifier_(false) {}

template <typename PointT>
pcl::people::PersonClassifier<PointT>::~PersonClassifier () {}

template <typename PointT> bool
pcl::people::PersonClassifier<PointT>::loadSVMFromFile (std::string svm_filename)
{
  std::string line;
  std::ifstream SVM_file;
  SVM_file.open(svm_filename.c_str());

  getline (SVM_file,line);      // read window_height line
  size_t tok_pos = line.find_first_of(":", 0);  // search for token ":"
  window_height_ = std::atoi(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read window_width line
  tok_pos = line.find_first_of(":", 0);  // search for token ":"
  window_width_ = std::atoi(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read SVM_offset line
  tok_pos = line.find_first_of(":", 0);  // search for token ":"
  SVM_offset_ = std::atof(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read SVM_weights line
  tok_pos = line.find_first_of("[", 0);  // search for token "["
  size_t tok_end_pos = line.find_first_of("]", 0);  // search for token "]" , end of SVM weights
  size_t prev_tok_pos;
  while (tok_pos < tok_end_pos) // while end of SVM_weights is not reached
  {
    prev_tok_pos = tok_pos;
    tok_pos = line.find_first_of(",", prev_tok_pos+1);  // search for token ","
    SVM_weights_.push_back(std::atof(line.substr(prev_tok_pos+1, tok_pos-prev_tok_pos-1).c_str()));
  }
  SVM_file.close();
  
  if (SVM_weights_.size() == 0)
  {
    PCL_ERROR ("[pcl::people::PersonClassifier::loadSVMFromFile] Invalid SVM file!\n");
    return (false);
  }
  else
  {
    return (true);
  }
}

template <typename PointT> void
pcl::people::PersonClassifier<PointT>::setSVM (int window_height, int window_width, std::vector<float> SVM_weights, float SVM_offset)
{
  window_height_ = window_height;
  window_width_ = window_width;
  SVM_weights_ = SVM_weights;
  SVM_offset_ = SVM_offset;
}

template <typename PointT> void
pcl::people::PersonClassifier<PointT>::enableRGBFileExport (bool enable_rgb_file_export)
{
    enable_rgb_file_export_ = enable_rgb_file_export;
}

template <typename PointT> void
pcl::people::PersonClassifier<PointT>::getSVM (int& window_height, int& window_width, std::vector<float>& SVM_weights, float& SVM_offset)
{
  window_height = window_height_;
  window_width = window_width_;
  SVM_weights = SVM_weights_;
  SVM_offset = SVM_offset_;
}


template <typename PointT> void
pcl::people::PersonClassifier<PointT>::useOpenCVGpuClassifier(bool enable)
{
    use_opencv_gpu_classifier_ = enable;
    setSVM(128, 64, cv::gpu::HOGDescriptor::getPeopleDetector64x128(), 0.0);
}


template <typename PointT> double
pcl::people::PersonClassifier<PointT>::evaluate (cv::Mat& rgbImage,
              Eigen::Vector2f& bottom,
              Eigen::Vector2f& top,
              Eigen::Vector2f& centroid,
              cv::Mat& rgbImageVisualization,
              cv::Rect& personRect)
{
  float xc = centroid(0);
  float yc = centroid(1);
  float height_person = bottom(1) - top(1);

  if (SVM_weights_.size() == 0)
  {
      PCL_ERROR ("[pcl::people::PersonClassifier::evaluate] SVM has not been set!\n");
      return (-1000);
  }

  int height = floor((height_person * window_height_) / (0.75 * window_height_) + 0.5);  // floor(i+0.5) = round(i)
  int width = floor((height_person * window_width_) / (0.75 * window_height_) + 0.5);
  int xmin = floor(xc - width / 2 + 0.5);
  int ymin = floor(yc - height / 2 + 0.5);
  int xmax = xmin + width;
  int ymax = ymin + height;

  int img_xmin = 0;
  int img_ymin = 0;
  int img_xmax = rgbImage.cols - 1;
  int img_ymax = rgbImage.rows - 1;

  if(!rgbImageVisualization.empty()) {
      cv::circle(rgbImageVisualization, cv::Point(xc, yc), 5, cv::Scalar(0, 1, 1, 1), -1);
  }

  double confidence = 0.0;

  if (height > 0)
  {
    // Extract person ROI without crossing the RGB image limits
    cv::Rect safePersonRect(
        cv::Point(std::max(img_xmin, xmin), std::max(img_ymin, ymin)),
        cv::Point(std::min(img_xmax, xmax), std::min(img_ymax, ymax)));

    if(!rgbImageVisualization.empty()) {
        cv::rectangle(rgbImageVisualization, safePersonRect, cv::Scalar(0.0, 1.0, 0.0, 1.0));
    }

    personRect = safePersonRect; // used to publish rectangle later on if it's a detection
    cv::Mat extractedPerson = rgbImage(safePersonRect);

    // If near the border, fill with black
    int borderTop       = ymin < img_ymin ? -ymin : 0;
    int borderBottom    = ymax > img_ymax ? ymax - img_ymax : 0;
    int borderLeft      = xmin < img_xmin ? -xmin : 0;
    int borderRight     = xmax > img_xmax ? xmax - img_xmax : 0;

    cv::Mat extractedPersonWithBorder;
    cv::copyMakeBorder(extractedPerson, extractedPersonWithBorder, borderTop, borderBottom, borderLeft, borderRight, cv::BORDER_CONSTANT, cv::Scalar(0.0f, 0.0f, 0.0f));
    assert(extractedPersonWithBorder.cols == width && extractedPersonWithBorder.rows == height);

    // Resize image to match window size used in the training stage:
    int sampleWidth = window_width_, sampleHeight = window_height_;
    //if(window_height_ < 120) {
    //    sampleWidth = TODO: Test to use small-window classifier
    //}

    cv::Mat sample;
    cv::resize(extractedPersonWithBorder, sample, cv::Size(window_width_, window_height_), 0, 0, cv::INTER_LINEAR);

    // Export image to file
    if(enable_rgb_file_export_) {
        static int personRgbCounter = 0;
        std::stringstream imageFilename; imageFilename << "person_rgb_" << personRgbCounter++ << ".png";
        cv::Mat outputSample; sample.convertTo(outputSample, CV_8UC3, 255.0);
        cv::imwrite(imageFilename.str(), outputSample);
    }

    if(!use_opencv_gpu_classifier_) // original CPU-based classifier
    {
        // Convert the OpenCV image (in BGR format) to array of float (complete R channel - complete G channel - complete B channel):
        // FIXME: This could be sped up
        float* sample_float = new float[sample.rows * sample.cols * sample.channels()];
        int delta = sample.cols * sample.rows;

        for(int row = 0; row < sample.rows; row++)
        {
          for(int col = 0; col < sample.cols; col++)
          {
            // NOTE: Dimensions x and y are being swapped here (left side). It was like this in the original implementation.
            sample_float[row + sample.rows * col + delta * 0] = sample.at<cv::Vec3f>(row, col)[2]; // B
            sample_float[row + sample.rows * col + delta * 1] = sample.at<cv::Vec3f>(row, col)[1]; // G
            sample_float[row + sample.rows * col + delta * 2] = sample.at<cv::Vec3f>(row, col)[0]; // R
          }
        }

        // Calculate HOG descriptor:
        pcl::people::HOG hog;
        float *descriptor = (float*) calloc(SVM_weights_.size(), sizeof(float));
        hog.compute(sample_float, descriptor);

        // Calculate confidence value by dot product:
        for(unsigned int i = 0; i < SVM_weights_.size(); i++)
        {
          confidence += SVM_weights_[i] * descriptor[i];
        }

        // Confidence correction:
        confidence -= SVM_offset_;

        delete[] descriptor;
        delete[] sample_float;
    }
    else // GPU-based classifier
    {
        // NOTE: SVM parameters can still be set externally using setSVM(). Use e.g. cv::gpu::HOGDescriptor::getPeopleDetector64x128();
        const bool gammaCorrectionEnabled = false;
        const int numLevels = 16;

        static cv::gpu::HOGDescriptor* gpu_hog = NULL;
        if(NULL == gpu_hog) {
            gpu_hog = new cv::gpu::HOGDescriptor(cv::Size(window_width_, window_height_), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9,
                cv::gpu::HOGDescriptor::DEFAULT_WIN_SIGMA, 0.2, gammaCorrectionEnabled, numLevels);

            gpu_hog->setSVMDetector(SVM_weights_);
        }

        cv::Mat sampleInBytes, convertedSample;
        sample.convertTo(sampleInBytes, CV_8UC3, 255.0);
        cv::cvtColor(sampleInBytes, convertedSample, cv::COLOR_BGR2BGRA);

        cv::gpu::GpuMat gpu_img;
        gpu_img.upload(convertedSample);

        std::vector<cv::Point> hits, locations; // hits are locations with confidence above classifier's default threshold
        std::vector<double> confidences;
        gpu_hog->computeConfidence(gpu_img, hits, SVM_offset_, cv::Size(window_width_, window_height_), cv::Size(0, 0), locations, confidences);

        assert(confidences.size() == 1);
        confidence = confidences[0];
    }

    if(!rgbImageVisualization.empty()) {
        std::stringstream confidenceStr; confidenceStr << std::fixed << std::setprecision(3) << confidence;
        int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontScale = 0.4;
        int thickness = 1;
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(confidenceStr.str(), fontFace, fontScale, thickness, &baseline);
        cv::Point textOrg(safePersonRect.x + (safePersonRect.width  - textSize.width)/2,
                          safePersonRect.y - textSize.height);
        cv::putText(rgbImageVisualization, confidenceStr.str(), textOrg, fontFace, fontScale, cv::Scalar(0.0, 1.0, 0.0, 1.0), thickness);
    }
  }
  else
  {
    confidence = std::numeric_limits<double>::quiet_NaN();
  } 

  return confidence;
}
#endif /* PCL_PEOPLE_PERSON_CLASSIFIER_HPP_ */
