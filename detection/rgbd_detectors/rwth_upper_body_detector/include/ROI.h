#ifndef ROI_H
#define ROI_H

#include "Matrix.h"
#include "Vector.h"
#include "Camera.h"
#include "QImage"
#include "pointcloud.h"

class ROI
{
public:
    ROI();
//    ROI(Vector<Vector<double> > &region);

    inline int width() const { return width_; }
    inline int height() const { return height_; }
    inline int center_y() const { return center_y_; }


    // static methods
    static void ExtractROIs(Vector<ROI>& all_ROIs, const Matrix<int> &labeled_ROIs, int number_of_ROIs);
    bool GetRegion2D(Vector<double>& bbox, const Camera& camera, const PointCloud& point_cloud);

    int ind_min_x, ind_max_x;
    int ind_min_y, ind_max_y;
    int ind_min_z, ind_max_z;
    bool has_any_point;
protected:
    inline void Compute();

    int width_;
    int height_;
    int center_y_;
    int min_x, max_x, min_y, max_y;
};


#endif // ROI_H
