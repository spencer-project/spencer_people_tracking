#ifndef ANCILLARYMETHODS_DENNIS_H
#define ANCILLARYMETHODS_DENNIS_H

//#include <unistd.h>
//#include <sys/times.h>
//#include <sys/time.h>
//#include <termios.h>
//#include <dirent.h>
//#include <errno.h>
//#include <sys/stat.h>

//#include <iostream>
//#include <string>
//#include <sstream>
//#include <algorithm>

#include "Vector.h"
#include "Matrix.h"
#include "Globals.h"
#include "Camera.h"
#include "Hypo.h"

#include <ros/ros.h>

class AncillaryMethods
{

public:
    static double DistanceToPlane(const Vector<double> &point, const Vector<double> &gp);

    static void MinMaxVecVecDim(Vector<Vector<double> > &v, int second_dim, double& min, double& max);
    static void MinMaxIndexVecVecDim(Vector<Vector<double> > &v, int second_dim, int& index_min, int& index_max);
    static double MinVecVecDim(Vector<Vector<double> > &v, int second_dim, int& index);
    static double MaxVecVecDim(Vector<Vector<double> > &v, int second_dim, int& index);

    static double LevelOneIOU(const Matrix<double> &temp, const Vector<double> &bbox1, const Vector<double> &bbox2, const Vector<double> &rectIntersect);
    static void IntersetRect(const Vector<double> &rect1, const Vector<double> &rect2, Vector<double>& rectInter);

    //***********************************************************
    // METHODS FOR THE UPPER BODY DETECTOR
    //***********************************************************
    static void NonMinSuppression2d(Matrix<double> &distances, Vector<Vector<double> >& pos_max, double threshold);
    static void GreedyNonMaxSuppression(Vector<Vector<double> > &all_boxes, double overlap_threshold, double filter_thresh, const Matrix<double> &temp,
                                                           Vector<Vector<double> > &final_bboxes);

    static Vector<double> PlaneToWorld(const Camera &camera, const Vector<double> &plane_in_camera);
    static Vector<double> PlaneToCam(const Camera &camera);

    static Matrix<double> GetDepthLibelas(int frame_nr, const Camera &camera, double baseline);
    static Camera GetCameraOrigin(const Camera &camWorld);

    static double  MedianOfMatrixRejectZero(const Matrix<double> &mat, int start_row, int end_row, int start_col, int end_col);


    //***********************************************************
    // Visulization
    //***********************************************************
//    static void RenderBBox2DWithScore(const Vector<double> &bbox, QImage& image, int r, int g, int b, int lineWidth);


    static Vector<double> getGaussian1D(double sigma, double precision);
    static Vector<double> conv1D(const Vector<double>& vec, const Vector<double>& kernel);

    static void MorphologyErode(Matrix<double> &img);
    static void MorphologyDilate(Matrix<double> &img);
    static void MorphologyOpen(Matrix<double> &img);


    //***********************************************************
    // Given a pos in 2D (image) and the camera par, project this 2D pos to the GP
    //***********************************************************
    static Vector<double> backprojectGP(const Vector<double>& pos2D, const Camera& cam, const Vector<double>& gp);

    //***********************************************************
    // Return size of the inlier struct
    //***********************************************************
    static int getSizeIdx(Vector < FrameInlier >& idx);
    //***********************************************************
    // Intersection of two sets of Inliers
    //***********************************************************
    static void intersectIdx(Vector< FrameInlier >& idx1, Vector< FrameInlier >& idx2, Vector< FrameInlier >& intersection);

    static Matrix<double> smoothTrajMatrix(Matrix<double>& mat, int nSmoothSize);

    //***********************************************************
    // Swap the content of the Vector 1..n to n..1
    //***********************************************************
    static void swapVectorVolume(Vector<Volume <double> >& src);
    static void swapVectorMatrix(Vector<Matrix <double> >& src);

    //***********************************************************
    // Get Rectangle from direction and orientation
    //***********************************************************
    static void compute_rectangle(Vector<double>& main4D, Vector<double>& ort4D, Vector<double>& Lmax, Vector<double>& x, Matrix<double>& result);


    ///////////////// Segmentation part /////////////////////////////////////////
    static Matrix<double> conv1D(Matrix<double> &im, Vector<double> &kernel, bool dirFlag);


    ////////// LM & Curve Detectors ////////////////////////////////////////////
    static void ExtractSlopsOnBorder(const Matrix<double>& image, Vector<double>& ys, Vector<double>& slopes, int start_x = 0, int end_x = -1, int start_y = 0, int end_y = -1);
    static Vector<int> FindLocalMax(Vector<double>& slopes);
    static void ExtractBorder(const Matrix<double>& image, Vector<double>& ys, int start_x = 0, int end_x = -1, int start_y = 0, int end_y = -1);
};



#endif // ANCILLARYMETHODS_DENNIS_H

