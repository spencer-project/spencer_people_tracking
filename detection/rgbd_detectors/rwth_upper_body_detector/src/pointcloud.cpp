#include "pointcloud.h"

PointCloud::PointCloud(const Camera &camera, const Matrix<double> &depth_map)
    : X(depth_map.total_size(), -1), Y(depth_map.total_size(), -1), Z(depth_map.total_size(), -1)
{
    number_of_points = 0;
    int width = depth_map.x_size();

    double c20 = camera.K()(2,0);
    double c00 = camera.K()(0,0);
    double c21 = camera.K()(2,1);
    double c11 = camera.K()(1,1);

    for(int i=0; i<depth_map.total_size(); ++i)
    {
        double z = depth_map.data()[i];
        if(z>0.1 && z<Globals::freespace_max_depth_to_cons)
        {
            X[i] = z*(((i%width)-c20)/c00);
            Y[i] = z*(((i/width)-c21)/c11);
            Z[i] = z;
            ++number_of_points;
        }
    }
}
