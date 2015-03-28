#include "Camera.h"
#include "Globals.h"

Camera::Camera()
{
    
}

Camera::Camera(const Matrix<double>& K, const Matrix<double>& R, const Vector<double>& t, const Vector<double>& GP)
{
    this->K_ = K;
    this->R_ = R;
    this->t_ = t;
    this->GP_ = GP;

    Precompute();
}

Camera::Camera(const char *filename, int central_flag)
{
    int CamIntStart = 0;
    int CamIntEnd = 9;

    int CamRotStart = 12;
    int CamRotEnd = 21;

    int CamPosStart = 21;
    int CamPosEnd = 24;

    int GPStart = 24;
    int GPEnd = 28;

    Vector<double> completeData;
    if (central_flag == 1)
    {
        completeData.readTXT(filename, 28);

        //*********************************************************
        // Read in Camera inter parameter
        //*********************************************************
        K_.set_size(3, 3);
        for (int i = CamIntStart; i < CamIntEnd; i++) {
            K_.data()[i - CamIntStart] = completeData(i);
        }

        //*********************************************************
        // Read in Camera Rotation parameter
        //*********************************************************
        R_.set_size(3, 3);
        for (int i = CamRotStart; i < CamRotEnd; i++) {
            R_.data()[i - CamRotStart] = completeData(i);
        }

        //*********************************************************
        // Read in Camera Position parameter
        //*********************************************************
        t_.setSize(3);
        for (int i = CamPosStart; i < CamPosEnd; i++)
        {
            t_(i - CamPosStart) = completeData(i);
        }

        //*********************************************************
        // Read in Ground Plane Parameter
        //*********************************************************
        GP_.setSize(4);
        for (int i = GPStart; i < GPEnd; i++){
            GP_(i - GPStart) = completeData(i);
        }
        GP_(3) = GP_(3) * (-1.0);
    }

    Precompute();
}

void Camera::Precompute()
{
    //*********************************************************
    //Compute m_mKRt
    //*********************************************************
    KRt_ = K_ * Transpose(R_);

    //*********************************************************
    //Compute KRtT
    //*********************************************************
    KRtT_ = KRt_ * t_;

    //*********************************************************
    //Compute VPN
    //*********************************************************

    VPN_ = R_.getColumn(2);

    //*********************************************************
    //Compute VPd
    //*********************************************************

    VPd_ = 0.0;

    for(int i = 0; i < t_.getSize(); i++)
    {
        VPd_ += t_(i)*(VPN_(i)*(-1.0));
    }

    //*********************************************************
    //Compute GPN and GPD
    //*********************************************************
    GPD_ = GP_(3);
    GPN_.setSize(3);

    for(int i = 0; i < GPN_.getSize(); i++)
    {
        GPN_(i) = GP_(i);
    }
}

void Camera::WorldToImage(const Vector<double>& pos3D, double world_scale, Vector<double>& pos2D) const
{


//    cout << "Rotation " << endl;
//    R_.Show();
//    cout << "INt " << endl;
//    K_.Show();
//    cout << "pos" << endl;
//    t_.show();

//    cout << "pos3D " << endl;
//    pos3D.show();

    Vector<double> vKrt = KRt_ * pos3D * (1.0/world_scale);
    vKrt -=(KRtT_);


//    vKrt.show();

    pos2D.setSize(3);
    if(vKrt(2) == 0)
    {
        ROS_DEBUG("'Warning in WorldToImage(): transformed point is invalid!'");
//        if(Globals::verbose)
//            cout <<"'Warning in WorldToImage(): transformed point is invalid!'" << endl;
    }else
    {
        pos2D(0) = (vKrt(0)/vKrt(2));
        pos2D(1) = (vKrt(1)/vKrt(2));
        pos2D(2) = 1.0;
    }
}

void Camera::ProjectToGP(const Vector<double>& pos3D, double world_scale, Vector<double>& pos3D_on_plane) const
{
    double d = DotProduct(GPN_, pos3D) + GPD_*world_scale;
    pos3D_on_plane = pos3D - GPN_ * d;
}

// Modified to R and t instead of Rt and Rt*t
void Camera::getRay(Vector<double>& x, Vector<double>& ray1, Vector<double>& ray2)
{
    Matrix<double> rot = get_R();
    Matrix<double> Kinv = get_K();
//    Vector<double> pos = get_t();

    ray1 = t_;

    Kinv.inv();

    rot *= Kinv;
    ray2 = rot * x;
    ray2 += ray1;
}

double Camera::bbToDetection(const Vector<double>& bbox, Vector<double>& pos3D, double ConvertScale, double& dist)
{
//    pos3D.clearContent();
    pos3D.setSize(3);

    double x = bbox(0);
    double y = bbox(1);
    double w = bbox(2);
    double h = bbox(3);

    // bottom_left and bottom_right are the point of the BBOX
    Vector<double> bottom_left(3, 1.0);
    bottom_left(0) = x + w/2.0;
    bottom_left(1) = y + h;

    Vector<double> bottom_right(3, 1.0);
    bottom_right(0) = x + w;
    bottom_right(1) = y + h;

    Vector<double> ray_bot_left_1;
    Vector<double> ray_bot_left_2;

    Vector<double> ray_bot_right_1;
    Vector<double> ray_bot_right_2;

    // Backproject through base point
    getRay(bottom_left, ray_bot_left_1, ray_bot_left_2);
    getRay(bottom_right, ray_bot_right_1, ray_bot_right_2);

    Vector<double> gpPointLeft;
    Vector<double> gpPointRight;

    // Intersect with ground plane
    intersectPlane(GPN_, GPD_, ray_bot_left_1, ray_bot_left_2, gpPointLeft);
    intersectPlane(GPN_, GPD_, ray_bot_right_1, ray_bot_right_2, gpPointRight);

    // Find top point
    Vector<double> ray_top_1;
    Vector<double> ray_top_2;

    Vector<double> aux(3, 1.0);
    aux(0) = x;
    aux(1) = y;

    getRay(aux, ray_top_1, ray_top_2);

    // Vertical plane through base points + normal
    Vector<double> point3;
    point3 = gpPointLeft;
    point3 -= (GPN_);
    Vector<double> vpn(3,0.0);
    Vector<double> diffGpo1Point3;
    Vector<double> diffGpo2Point3;

    diffGpo1Point3 = gpPointLeft;
    diffGpo1Point3 -=(point3);

    diffGpo2Point3 = gpPointRight;
    diffGpo2Point3 -= point3;

    vpn = cross(diffGpo1Point3,diffGpo2Point3);
    double vpd = (-1.0)*DotProduct(vpn, point3);

    Vector<double> gpPointTop;
    intersectPlane(vpn, vpd, ray_top_1, ray_top_2, gpPointTop);

    // Results
    gpPointTop -= gpPointLeft;

    // Compute Size
    double dSize = gpPointTop.norm();

    // Compute Distance
    aux = t_;
    aux -= gpPointLeft;
    dist = aux.norm();

    dist = dist * ConvertScale;
    if(gpPointLeft(2) < t_(2)) dist = dist * (-1.0);
    // Compute 3D Position of BBOx
    double posX = gpPointLeft(0) * ConvertScale;
    double posY = gpPointLeft(1) * ConvertScale;
    double posZ = gpPointLeft(2) * ConvertScale;

    pos3D(0) = (posX);
    pos3D(1) = (posY);
    pos3D(2) = (posZ);

    return dSize * ConvertScale;

}

void Camera::intersectPlane(Vector<double>& gp, double gpd, Vector<double>& ray1, Vector<double>& ray2, Vector<double>& point)
{
    Vector<double> diffRay;
    diffRay = ray1;
    diffRay -= ray2;

    double den = DotProduct(gp, diffRay);
    double t = (DotProduct(gp, ray1) + gpd) / den;

    point = ray1;
    diffRay = (ray2);
    diffRay -= (ray1);
    diffRay *= t;
    point += diffRay;
}

void Camera::jacFor3DCov(Vector<double>& X, Matrix<double>& Cov)
{
    Matrix<double> R = Transpose(R_);
    Matrix<double> K = K_;
    Vector<double> t = t_;

//    cout << "Matrix R" << endl;
//    R.show();
//    cout << "position " << endl;
//    X.show();
//    cout << "camera position" << endl;
//    t.show();

    double denom = (R(2,0)*(X(0)-t(0))+R(2,1)*(X(1)-t(1))+R(2,2)*(X(2)-t(2)))*
                               (R(2,0)*(X(0)-t(0))+R(2,1)*(X(1)-t(1))+R(2,2)*(X(2)-t(2)));

    double num1 = K(0,0) * ((X(1)-t(1)) * (R(0,0) * R(2,1) - R(2,0) * R(0,1)) +
                     (X(2)-t(2)) * (R(0,0) * R(2,2) - R(0,2) * R(2,0)));

    double num2 = K(0,0) * ((X(0)-t(0)) * (R(0,1) * R(2,0) - R(0,0) * R(2,1)) +
                     (X(2)-t(2)) * (R(0,1) * R(2,2) - R(2,1) * R(0,2)));

    double num3 = K(0,0) * ((X(0)-t(0)) * (R(0,2) * R(2,0) - R(0,0) * R(2,2)) +
                     (X(1)-t(1)) * (R(0,2) * R(2,1) - R(2,2) * R(0,1)));

    double num4 = K(1,1) * ((X(1)-t(1)) * (R(1,0) * R(2,1) - R(2,0) * R(1,1)) +
                     (X(2)-t(2)) * (R(1,0) * R(2,2) - R(2,0) * R(1,2)));

    double num5 = K(1,1) * ((X(0)-t(0)) * (R(1,1) * R(2,0) - R(2,1) * R(1,0)) +
                     (X(2)-t(2)) * (R(1,1) * R(2,2) - R(2,1) * R(1,2)));

    double num6 = K(1,1) * ((X(0)-t(0)) * (R(1,2) * R(2,0) - R(2,2) * R(1,0)) +
                     (X(1)-t(1)) * (R(1,2) * R(2,1) - R(2,2) * R(1,1)));

    Cov.set_size(3,2, 0.0);

    if(denom != 0)
    {
        Cov(0,0) = num1/denom;
        Cov(1,0) = num2/denom;
        Cov(2,0) = num3/denom;
        Cov(0,1) = num4/denom;
        Cov(1,1) = num5/denom;
        Cov(2,1) = num6/denom;
    }
    else
    {
        //        R.show();
        //        cout << "X" << endl;
        //        X.show();
        //        cout << "t" << endl;
        //        t.show();

        //assert(false);
        //cout << "WARNING: The Jacobians for 3D Covariance is undefiend" << endl;
        Cov(0,0) = 1;
        Cov(1,0) =1;
        Cov(2,0) = 1;
        Cov(0,1) = 1;
        Cov(1,1) =1;
        Cov(2,1) =1;
    }
}

bool Camera::isPointInFrontOfCam(const Vector<double>& point) const
{
    // Check if a point is in front of the camera.

    if(DotProduct(point, VPN_) + VPd_ > 0){
        return true;
    }else{
        return false;
    }
}
