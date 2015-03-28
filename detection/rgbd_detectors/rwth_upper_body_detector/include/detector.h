#ifndef DETECTOR_H
#define DETECTOR_H

#include "Vector.h"
#include "Matrix.h"
#include "Camera.h"
#include "ROI.h"
#include "pointcloud.h"
#include "AncillaryMethods.h"

#include <ros/ros.h>

class Detector
{
public:
    Detector();

    bool visualize_roi;
    Matrix<int> roi_image;
    Matrix<int> labeledROIs;
    ////////////////////////////////////////////////////////////////////////
    // ProcessFrame:
    //      Processes a single frame and detects all upper bodies in a frame.
    //
    // prarameters:
    //      input:
    //          camera              -   Camera settings related to the frame.
    //          depth_map           -   Depth map image of the frame.
    //          upper_body_template -   Template of upper body.
    //      output:
    //          detected_bounding_boxes -   Resulting bounding boxes around
    //                                      detected upper bodies.
    ////////////////////////////////////////////////////////////////////////
    void ProcessFrame(const Camera& camera, const Matrix<double>& depth_map, const PointCloud& point_cloud,
                      const Matrix<double>& upper_body_template, Vector<Vector<double> >& detected_bounding_boxes);

    void ComputeFreespace(const Camera &camera, Matrix<int> &occ_map_binary,
                          Matrix<int> &mat_2D_pos_x, Matrix<int> &mat_2D_pos_y, const PointCloud& point_cloud);
private:
    ///////////////////////////////////////////////////////////////////////
    // EvaluateTemplate:
    //      Slides template of upper body over Depth-Map only over
    //      Close-Range-Bounding-Boxes and returns bounding boxes around
    //      detected upperbodies.
    //
    // parameters:
    //      input:
    //          upper_body_template -   template of upper body
    //          depth_map           -   Depth map image
    //          close_range_BBoxes  -   Bounding boxes over ROIs
    //          distances           -
    ///////////////////////////////////////////////////////////////////////
    Vector<Vector<double> > EvaluateTemplate(const Matrix<double> &upper_body_template, const Matrix<double> &depth_map,
                                             Vector<Vector<double> > &close_range_BBoxes, Vector<Vector<double> > distances);

    ///////////////////////////////////////////////////////////////////////
    // PreprocessROIs:
    //
    // parameters:
    //      input:
    //          labeled_ROIs    -   binary occ_map
    //      output:
    //          labeled_ROIs    -   Labeled ROIs
    //          all_ROIs        -   All ROIs
    ///////////////////////////////////////////////////////////////////////
    void PreprocessROIs(Matrix<int>& labeled_ROIs, Vector<ROI>& all_ROIs);

    ///////////////////////////////////////////////////////////////////////
    // ExtractPointsInROIs:
    //      Extracts all 2D points inside ROIs.
    //
    // parameters:
    //      input:
    //          mat_2D_pos_x    -
    //          mat_2D_pos_y    -
    //          mat_3D_points_x -
    //          mat_3D_points_y -
    //          mat_3D_points_z -
    //          labeled_ROIs    -
    //      output:
    //          all_points_in_ROIs  -
    ///////////////////////////////////////////////////////////////////////
    void ExtractPointsInROIs(Vector<ROI> &all_ROIs,
                             const Matrix<int>& mat_2D_pos_x, const Matrix<int>& mat_2D_pos_y,
                             const PointCloud& point_cloud,
                             const Matrix<int>& labeled_ROIs);

};

#endif // DETECTOR_H
