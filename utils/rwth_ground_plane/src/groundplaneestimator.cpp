#include "groundplaneestimator.h"
#include "Math.h"

GroundPlaneEstimator::GroundPlaneEstimator()
{
}

Vector<double> GroundPlaneEstimator::ComputeGroundPlane(const PointCloud& point_cloud)
{
    Vector<int> allPointsOnGPIndices;
    ComputePointsOnGP(point_cloud, allPointsOnGPIndices);

    int n_points = allPointsOnGPIndices.getSize();
    int nr_samples = 200;
    Vector<Vector<double> > newPoints(nr_samples);

    Vector<double> p(3);

    if(n_points<nr_samples)
    {
        newPoints.resize(n_points);
        for(int i = 0; i<n_points; ++i)
        {
            int ind = allPointsOnGPIndices(i);
            p(0) = point_cloud.X(ind); p(1) = point_cloud.Y(ind); p(2) = point_cloud.Z(ind);
            newPoints(i) = p;
        }
    }
    else
    {
        vector< bool > sel(n_points, false);

        for(int i = 0; i < nr_samples; i++)
        {
            int r;
            do {
                r = int(rand() % n_points);
            } while (sel[r]);
            sel[r] = true;

            int ind = allPointsOnGPIndices(r);
            p(0) = point_cloud.X(ind); p(1) = point_cloud.Y(ind); p(2) = point_cloud.Z(ind);
            newPoints(i) = p;
        }
    }

    Vector<double> gp = lmsPlane(newPoints, Globals::nrInter_ransac);


    Vector<double> normal(gp(0), gp(1), gp(2));
    double factor = sqrt(normal(0)*normal(0) + normal(1)*normal(1) + normal(2)*normal(2));

    gp *= 1.0/factor;
    if(gp(1) > 0)
    {
        gp *= -1;
    }

    return gp;
}

void GroundPlaneEstimator::ComputePointsOnGP(const PointCloud& point_cloud, Vector<int>& allPointsOnGp3DIndex)
{
    double scaleZ = 10;
    double scaleX = 10;

    double minX = -5;
    double minZ = 0;

    double maxX = 5;
    double maxZ = 10;


    ///////////////////////////////////
    double minZ_up = 0;

    int xbins = round(maxX*scaleX-minX*scaleX);
    int zbins = round(maxZ*scaleZ-minZ_up*scaleZ);

    Matrix<double> histR(xbins+1, zbins+1, 0.0);

    double stepX = (maxX - minX)/double (xbins);
    double stepZ = (maxZ - minZ_up)/double (zbins);

    Vector<Vector<Vector<int > > > pointsInBinsIndex(xbins+1);
    for(int i = 0; i < xbins+1; i++)
    {
        pointsInBinsIndex(i).setSize(zbins+1);
    }

//    double log_2 = log(2);

    for(int i = 0; i < point_cloud.X.getSize(); i++)
    {
        if(point_cloud.Z(i) <= 0)
            continue;

        double x = point_cloud.X(i) - minX;
        double z = point_cloud.Z(i) - minZ_up;

        int posX = round(x / stepX);
        int posZ = round(z / stepZ);

        if(posX<0 || posX>=xbins || posZ<0 || posZ>=zbins)
            continue;

//        histR(posX, posZ) += max(1.0, z*(log(z) / log_2));
        histR(posX, posZ) += max(1.0, z);
        pointsInBinsIndex(posX)(posZ).pushBack(i);
    }
    ///////////////////////////////////

    for(int i = 0; i < histR.x_size(); i++)
    {
        for(int j = 0; j < histR.y_size(); j++)
        {
            if(histR(i,j) < Globals::numberOfPoints_reconAsObstacle)
            {
                for(int l = 0; l < pointsInBinsIndex(i)(j).getSize(); l++)
                {
                    int ind = pointsInBinsIndex(i)(j)(l);
                    if(point_cloud.Y(ind) > 0)
                        allPointsOnGp3DIndex.pushBack(ind);
                }
            }
        }
    }
}

Vector<double> GroundPlaneEstimator::lmsPlane(Vector<Vector<double> >& pts, int nrIter)
{
    Vector<double> bestpl;
    int bests[3];
    double minerr = 1e9;

    if (pts.getSize() < 10)
        return Vector<double>(4, 0.0);

    bests[0] = -1;

    // Do LMS by sampling
    for (int iter = 0; iter < nrIter; iter++) {
        vector< bool > sel(pts.getSize(), false);

        // Generate test plane
        int ss[3];
        for (int j = 0; j < 3; j++) {
            int r;
            do {
                r = int(double(rand()) / RAND_MAX * (pts.getSize() - 1));
            } while (sel[r]);
            ss[j] = r;
            sel[r] = true;
        }

        Vector<double> gen_plane;
        gen_plane = construct_plane(pts[ss[0]], pts[ss[1]], pts[ss[2]]);
        gen_plane *= 1.0 / gen_plane.norm();

        // Calculate inliers
        vector< pair< double, int > > ds(pts.getSize());
        double dist;
        pair< double, int > ind_pair;
        for (int j = 0; j < pts.getSize(); j++)
        {
            dist = (gen_plane(0)*pts[j][0] + gen_plane(1)*pts[j][1] + gen_plane(2)*pts[j][2] + gen_plane(3));// /(sqrt(gen_plane(0)*gen_plane(0) + gen_plane(1)*gen_plane(1) + gen_plane(2)*gen_plane(2)));
            ind_pair.first = fabs(dist);
            ind_pair.second = j;
            ds[j] = ind_pair;
        }
        sort(ds.begin(), ds.end());

        double err = 0;
        for (unsigned j = 0; j < ds.size() / 2; j++)
            err += ds[j].first;

        if (err < minerr) {
            minerr = err;
            bests[0] = ss[0];
            bests[1] = ss[1];
            bests[2] = ss[2];
        }
    }

    if (bests[0] == -1) {
        cout << "No solution found." << endl;
        return Vector<double>(0, 0, 0, 1);
    }

    // --- LSQ fit of plane ---

    // Calculate LSQ-fit plane
    Vector<double> gen_plane;
    gen_plane = construct_plane(pts[bests[0]], pts[bests[1]], pts[bests[2]]);
    gen_plane *= 1.0 / gen_plane.norm();

    // Calculate inliers
    vector< pair< double, int > > ds(pts.getSize());
    for (int j = 0; j < pts.getSize(); j++) {
        Vector<double> pt(pts[j][0], pts[j][1], pts[j][2]);
        double d = gen_plane(0)*pt(0) + gen_plane(1)*pt(1) + gen_plane(2)*pt(2) + gen_plane(3);
        ds[j] = (make_pair(fabs(d), j));
    }
    sort(ds.begin(), ds.end());

    Vector< Vector<double> > bestp(ds.size()/2);
    for (unsigned j = 0; j < ds.size() / 2; j++)
        bestp(j) = pts[ds[j].second];

    bestpl = LSQ(bestp);

    return bestpl;
}

Vector<double> GroundPlaneEstimator::construct_plane(Vector<double>& pt1, Vector<double>& pt2, Vector<double>& pt3) {


    Vector<double> result_plane(4);
    Vector<double> p13 = pt1 - pt3;
    Vector<double> p23 = pt2 - pt3;
    p13.cross(p23);

    result_plane(0) = p13(0);
    result_plane(1) = p13(1);
    result_plane(2) = p13(2);
    result_plane(3) = -DotProduct(p13, pt3);
    return result_plane;
}

Vector<double> GroundPlaneEstimator::LSQ(Vector<Vector<double> >& pts)
{
    // Solve LSQ s.t. ||Mu - u01|| = minimal
    Matrix<double> M(3, pts.getSize());
    Vector<double> m_mean(3, 0.0);


    // Fill M, calculate m_mean
    for (int i = 0; i < pts.getSize(); i++) {
        M.insertRow(pts(i), i);
        m_mean += pts(i);
    }

    m_mean *= 1.0/pts.getSize();

    for (int i = 0; i < pts.getSize(); i++) {
        M(0,i) -= m_mean[0];
        M(1,i) -= m_mean[1];
        M(2,i) -= m_mean[2];
    }

    Matrix<double> S(M.x_size(),M.x_size(), 0.0);
    Matrix<double> V(M.x_size(),M.y_size(), 0.0);

    Math::svd(M, S, V, true, 30);

    Vector<double> p(4);
    p[0] = V(2,0);
    p[1] = V(2,1);
    p[2] = V(2,2);
    p[3] = -(m_mean(0)*p(0) + m_mean(1)*p(1) + m_mean(2)*p(2));

    return p;
}
