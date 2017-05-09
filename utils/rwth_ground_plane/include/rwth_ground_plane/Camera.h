/* 
 * File:   Camera.h
 * Author: mitzel
 *
 * Created on July 6, 2009, 6:06 PM
 */

#ifndef _DENNIS_CAMERA_H
#define	_DENNIS_CAMERA_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>

#include "Vector.h"
#include "Matrix.h"

class Camera
{
public:

    //*******************************************************
    // Constructors
    //*******************************************************
    Camera();
    Camera(const char *filename, int central_flag);
    Camera(const Matrix<double> &K, const Matrix<double> &R, const Vector<double> &t, const Vector<double> &GP);

    //*******************************************************
    // Gettings
    //*******************************************************
    inline const Matrix<double>& K() const{ return K_;}
    inline Matrix<double> get_K() const{ return K_;}
    inline Matrix<double> get_R() const{ return R_;}
    inline Vector<double> get_t() const{return t_;}
    inline Vector<double> get_GP() const{return GP_;}
    inline Vector<double> get_GPN() const{return GPN_;}
    inline Vector<double> get_VPN() const{return VPN_;}
    inline Matrix<double> get_KRt() const{ return KRt_;}
    inline Vector<double> get_KRtT() const{ return KRtT_;}

    void WorldToImage(const Vector<double> &pos3D, double world_scale, Vector<double>& pos2D) const;
    void ProjectToGP(const Vector<double> &pos3D, double world_scale, Vector<double>& pos3D_on_plane) const;

    void getRay(Vector<double>& x, Vector<double>& ray1, Vector<double>& ray2);
    double bbToDetection(const Vector<double> &bbox, Vector<double>& pos3D, double ConvertScale, double& dist);
    void intersectPlane(Vector<double>& gp, double gpd, Vector<double>& ray1, Vector<double>& ray2, Vector<double>& point);
    void jacFor3DCov(Vector<double>& x3d, Matrix<double>& Cov);
    bool isPointInFrontOfCam(const Vector<double> &point) const;

private:

    // Transform the point to world coordinates
    void Precompute();

protected:
    Matrix<double> K_;  // Internal parameters
    Matrix<double> R_;  // Rotation parameters
    Vector<double> t_;  // Position parameters
    Vector<double> GP_; // Ground Plane parameters

    Vector<double> GPN_;
    double GPD_;

    Matrix<double> KRt_;

    Vector<double> KRtT_;

    Vector<double> VPN_;
    double VPd_;
};


#endif	/* _DENNIS_CAMERA_H */

