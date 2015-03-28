#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "Globals.h"
#include "Vector.h"
#include "Matrix.h"
#include "Camera.h"

class PointCloud
{
public:
    PointCloud(){}
    PointCloud(const Camera& camera, const Matrix<double>& depth_map);

    Vector<double> X, Y, Z;
    int number_of_points;
};

#endif // POINTCLOUD_H
