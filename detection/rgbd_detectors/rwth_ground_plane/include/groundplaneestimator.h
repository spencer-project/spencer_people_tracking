#ifndef GROUNDPLANEESTIMATOR_H
#define GROUNDPLANEESTIMATOR_H

#include "Camera.h"
#include "pointcloud.h"

class GroundPlaneEstimator
{
public:
    GroundPlaneEstimator();
    Vector<double> ComputeGroundPlane(const PointCloud &point_cloud);
private:
    void ComputePointsOnGP(const PointCloud& point_cloud, Vector<int>& allPointsOnGp3DIndex);
    Vector<double> lmsPlane(Vector<Vector<double> > &pts, int nrIter);
    Vector<double> construct_plane(Vector<double> &pt1, Vector<double> &pt2, Vector<double> &pt3);
    Vector<double> LSQ(Vector<Vector<double> > &pts);
};

#endif // GROUNDPLANEESTIMATOR_H
