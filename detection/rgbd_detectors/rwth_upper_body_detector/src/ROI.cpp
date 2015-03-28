#include "ROI.h"
#include "Globals.h"
#include "Vector.h"

#include <limits.h>

ROI::ROI()
{
    min_x = min_y = INT_MAX;
    max_x = max_y = INT_MIN;
    has_any_point = false;
}

//ROI::ROI(Vector< Vector <double> >&  region)
//{
//    double minX, maxX;
//    AncillaryMethods::MinMaxVecVecDim(region, 0, minX, maxX);
//    width_ = maxX - minX;

//    double minY, maxY;
//    AncillaryMethods::MinMaxVecVecDim(region, 1, minY, maxY);
//    height_ = maxY - minY;

//    // compute the center of mass on y-axis
//    center_y_ = minY + (height_+1)/2.0;
//}



void ROI::Compute()
{
    width_ = max_x - min_x;
    height_ = max_y - min_y;
    // compute the center of mass on y-axis
    center_y_ = min_y + (height_+1)/2.0;
}

void ROI::ExtractROIs(Vector<ROI>& all_ROIs, const Matrix<int>& labeled_ROIs, int number_of_ROIs)
{
//    Vector<Vector<Vector<double> > > regions(number_of_ROIs);

//    Vector<double> value(2);

    all_ROIs.setSize(number_of_ROIs);

    int labeled_ROIs_height = labeled_ROIs.y_size();
    ROI* roi;
    for(int i = 0; i < labeled_ROIs.x_size(); i++)
    {
        for(int j = 0; j < labeled_ROIs_height; j++)
        {
            if(labeled_ROIs(i,j) > 0)
            {
//                value(0) = i;
//                value(1) = j;
//                regions(labeled_ROIs(i,j)-1).pushBack(value);

                roi = &(all_ROIs(labeled_ROIs(i,j)-1));
                if(roi->min_x > i) roi->min_x = i;
                if(roi->min_y > j) roi->min_y = j;
                if(roi->max_x < i) roi->max_x = i;
                if(roi->max_y < j) roi->max_y = j;
            }
        }
    }

    for(int i=0; i<number_of_ROIs; ++i)
        all_ROIs(i).Compute();
//    all_ROIs.reserveCapacity(number_of_ROIs);

//    for(int i = 0; i < number_of_ROIs; i++)
//        all_ROIs.pushBack(ROI(regions(i)));
}

bool ROI::GetRegion2D(Vector<double>& bbox, const Camera& camera, const PointCloud &point_cloud/*Vector<Vector< double> >& points_inside_bins_of_indROI*/)
{
    // As it is required to estimate the scale of the 2D box based on the height of the bounding box, it is not sufficient to simply use the existing 3D points as the
    // person might be occluded by the image borders...


//    int index_min, index_max;

//    AncillaryMethods::MinMaxIndexVecVecDim(points_inside_bins_of_indROI, 0, index_min, index_max);
//    Vector<double> pos3DMinX = points_inside_bins_of_indROI(index_min);
//    Vector<double> pos3DMaxX = points_inside_bins_of_indROI(index_max);
    Vector<double> pos3DMinX(point_cloud.X(ind_min_x), point_cloud.Y(ind_min_x), point_cloud.Z(ind_min_x));
    Vector<double> pos3DMaxX(point_cloud.X(ind_max_x), point_cloud.Y(ind_max_x), point_cloud.Z(ind_max_x));

//    AncillaryMethods::MinMaxIndexVecVecDim(points_inside_bins_of_indROI, 1, index_min, index_max);
//    Vector<double> pos3DMinY = points_inside_bins_of_indROI(index_min);
//    Vector<double> pos3DMaxY = points_inside_bins_of_indROI(index_max);
    Vector<double> pos3DMinY(point_cloud.X(ind_min_y), point_cloud.Y(ind_min_y), point_cloud.Z(ind_min_y));
    Vector<double> pos3DMaxY(point_cloud.X(ind_max_y), point_cloud.Y(ind_max_y), point_cloud.Z(ind_max_y));

    Vector<double> pos2D_tmp(3);   // temporary variable

    camera.WorldToImage(pos3DMinX, Globals::WORLD_SCALE, pos2D_tmp);
    double minX = pos2D_tmp(0);

    camera.WorldToImage(pos3DMaxX, Globals::WORLD_SCALE, pos2D_tmp);
    double maxX = pos2D_tmp(0);

    camera.WorldToImage(pos3DMinY, Globals::WORLD_SCALE, pos2D_tmp);
    double minY = pos2D_tmp(1);

    camera.ProjectToGP(pos3DMaxY, Globals::WORLD_SCALE, pos3DMaxY);

    camera.WorldToImage(pos3DMaxY, Globals::WORLD_SCALE, pos2D_tmp);

    // Left
    bbox(0) = minX;
    // Top
    bbox(1) = minY;
    // Width
    bbox(2) = maxX-minX;
    // Height
    bbox(3) = pos2D_tmp(1)-minY;

    double incHeightValue = bbox(3)*Globals::inc_height_ratio;
    bbox(3) += incHeightValue;
    bbox(1) -= incHeightValue/2.0;

    double incWidthValue = bbox(2)*Globals::inc_width_ratio;
    bbox(2) += incWidthValue;
    bbox(0) -= incWidthValue/2.0;

    // Check Left
    if(bbox(0) >= Globals::dImWidth)
        return false;
    else if(bbox(0) < 0)
        bbox(0) = 0;

    // Check Top
    if(bbox(1) >= Globals::dImHeight)
        return false;
    else if(bbox(1) < 0)
        bbox(1) = 0;

    // Check Width
    if(bbox(2) <= 0)
        return false;
    else if(bbox(0)+bbox(2) >= Globals::dImWidth) // Check right
        bbox(2) = Globals::dImWidth - bbox(0) - 1;

    // Check Height
    if(bbox(3) <= 0)
        return false;
//    else if(bbox(3) >= Globals::dImHeight)
//        bbox(3) = Globals::dImHeight - 1;

    return true;
}
