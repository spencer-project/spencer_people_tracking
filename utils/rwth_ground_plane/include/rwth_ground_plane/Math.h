#ifndef MYMATH_H
#define MYMATH_H

#include "Matrix.h"
#include "Volume.h"
//#include <omp.h>

struct Point2D {
    double x, y;

    Point2D() { }

    Point2D(double x_, double y_) {
        x = x_;
        y = y_;
    }

    double det(const Point2D& pt) const {
        return x * pt.y - y * pt.x;
    }

    double normSq() const {
        return x * x + y * y;
    }

    double cross(const Point2D& p2, const Point2D& p3) const {
        return (p2.x - x) * (p3.y - y) - (p3.x - x) * (p2.y - y);
    }

    Point2D operator+(const Point2D& p) const {
        return Point2D(x + p.x, y + p.y);
    }

    Point2D operator-(const Point2D& p) const {
        return Point2D(x - p.x, y - p.y);
    }

    bool operator==(const Point2D& p) const {
        return (p.x == x && p.y == y);
    }
};

struct HomgPoint2D {
    double x, y, w;

    HomgPoint2D() { }

    HomgPoint2D(double x_, double y_, double w_) {
        x = x_;
        y = y_;
        w = w_;
    }

    HomgPoint2D cross(const HomgPoint2D& pt) const {
        return HomgPoint2D(y*pt.w - w*pt.y,
                           w*pt.x - x*pt.w,
                           x*pt.y - y*pt.x);
    }

    void normalise() {
        x /= w;
        y /= w;
        w = 1;
    }

    bool operator==(const HomgPoint2D& p) const {
        return (p.x == x && p.y == y && p.w == w);
    }
};

struct Point
{
    double x, y, z;
    Point() { }
    Point(double x_, double y_, double z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }
};

struct Quaternion
{
    double a, b, c, d;

    Quaternion() { }

    Quaternion(double a_, double b_, double c_, double d_)
    {
        a = a_;
        b = b_;
        c = c_;
        d = d_;
    }

    //quaternion multiplication
    Quaternion quat_mult(Quaternion q1, Quaternion q2)
    {
        Quaternion result;
        result.a = (q1.a*q2.a -q1.b*q2.b -q1.c*q2.c -q1.d*q2.d);
        result.b = (q1.a*q2.b +q1.b*q2.a +q1.c*q2.d -q1.d*q2.c);
        result.c = (q1.a*q2.c -q1.b*q2.d +q1.c*q2.a +q1.d*q2.b);
        result.d = (q1.a*q2.d +q1.b*q2.c -q1.c*q2.b +q1.d*q2.a);
        return result;
    }

    //Quaternion multiplication without the .a component
    Point quat_pointmult( Quaternion q1, Quaternion q2)
    {
        Point result;
        result.x = (q1.a*q2.b +q1.b*q2.a +q1.c*q2.d -q1.d*q2.c);
        result.y = (q1.a*q2.c -q1.b*q2.d +q1.c*q2.a +q1.d*q2.b);
        result.z = (q1.a*q2.d +q1.b*q2.c -q1.c*q2.b +q1.d*q2.a);

        return result;
    }

    //Uses quaternion mathematics to perform a rotation
    Point quat_rot(Point point, Point rotVec, double angle)
    {
        double sinCoeff;
        Quaternion rotQuat;
        Quaternion pointQuat;
        Quaternion conjQuat;
        Quaternion temp;

        sinCoeff=sin(angle*0.5);

        rotQuat.a = cos(angle*0.5);

        rotQuat.b=sinCoeff*rotVec.x;
        rotQuat.c=sinCoeff*rotVec.y;
        rotQuat.d=sinCoeff*rotVec.z;

        pointQuat.a =0;
        pointQuat.b = point.x;
        pointQuat.c = point.y;
        pointQuat.d = point.z;
        //calculate conjugate of the quaternion
        conjQuat.a = rotQuat.a;
        conjQuat.b = -rotQuat.b;
        conjQuat.c = -rotQuat.c;
        conjQuat.d = -rotQuat.d;

        //perform  rotation
        temp=quat_mult(rotQuat,pointQuat);
        point=quat_pointmult(temp,conjQuat);

        return point;
    }


};

//******************************************************
// Author D.Mitzel
//******************************************************
class Math
{
public:
    //*************************************************************************************************************************
    // Computes a singular value decomposition A=USV^T
    // Input: U MxN matrix
    // Output: U MxN matrix, S NxN diagonal matrix, V NxN diagonal matrix
    // Credits: T. Brox
    //*************************************************************************************************************************

    static void svd(Matrix<double>& U, Matrix<double>& S, Matrix<double>& V, bool aOrdering, int aIterations);

    //*************************************************************************************************************************
    // Cyclic Jacobi method for determining the eigenvalues and eigenvectors
    // of a symmetric matrix.
    // Ref.:  H.R. Schwarz: Numerische Mathematik. Teubner, Stuttgart, 1988. pp. 243-246.
    // Credits: T. Brox
    //*************************************************************************************************************************

    static void eigSymmMatrix(const Matrix<double>& aMatrix, Vector<double>& aEigenvalues, Matrix<double>& aEigenvectors);

    ///*************************************************************************************************************************
    // Method for intersection of two axis aligned rectangles
    // rect1 / rect2 is a vector of 4 doubles in the following order
    // x-Position, y-Positon width and height.
    //
    // Return is a boolean -> true if the rectangles intersect, false else.
    //**************************************************************************************************************************

    static bool intersectRects(Vector<double> rect1, Vector<double> rect2);

    ///*************************************************************************************************************************
    // This method computes the intersction of two axis aligned rectangels
    // returns the surface and rectInter will contain the intersection rectangle
    ///*************************************************************************************************************************

    static double intersetRectSurface(Vector<double>& rect1, Vector<double>& rect2, Vector<double>& rectInter);

    ///*************************************************************************************************************************
    // Bhattacharyya distance of two histograms
    ///*************************************************************************************************************************

    static double hist_bhatta(Volume<double> hist1, Volume<double> hist2);
    static double hist_bhatta(double* hist1, double* hist2, int size);

    ///*************************************************************************************************************************
    // PATransformation
    // Cyclic Jacobi method for determining the eigenvalues and eigenvectors
    // of a symmetric matrix.
    // Ref.:  H.R. Schwarz: Numerische Mathematik. Teubner, Stuttgart, 1988. pp. 243-246.
    // Credits: T. Brox
    ///*************************************************************************************************************************

    static void PATransformation(const Matrix<double>& aMatrix, Vector<double>& aEigenvalues, Matrix<double>& aEigenvectors);

    ///*************************************************************************************************************************
    // PABackTransformation
    // Credits: T. Brox
    ///*************************************************************************************************************************

    static void PABacktransformation(const Matrix<double>& aEigenvectors, const Vector<double>& aEigenvalues, Matrix<double>& aMatrix);

    static void invRegularized(Matrix<double>& src, double aMaxValue,double aMinValue);
    ///*************************************************************************************************************************
    // Evaluate Elipse
    ///*************************************************************************************************************************

    static bool evalElipse(double w, double h, double centerX, double centerY, double x, double y);
    static double evalElipseV(double h, double w, double centerX, double centerY, double x, double y);

    ///*************************************************************************************************************************
    // Compute the crossing point of two lines
    ///*************************************************************************************************************************

    static void crossLine(Vector<double> o1, Vector<double> r1, Vector<double> o2, Vector<double> r2, Vector<double>& crossPoint);

    static double dot(const Point2D& a, const Point2D& b);

    // Calculate the convex hull of a polygon (or just a set of points)
    // The coordinates in the convex hull will be in clockwise direction.
    static void convexHull(vector< Point2D >& v);

    static double area(const vector< Point2D >& points);

    static void calcLine(const Point2D& a1, const Point2D& a2, HomgPoint2D& l);

    /**
     * Find intersection point of two lines in general form
     * Returns nan if no intersection
     **/
    static Point2D intersect(const Point2D& a1, const Point2D& a2, const Point2D& b1, const Point2D &b2);

    static bool inside(const vector< Point2D >& v, const Point2D& pt);

    static double polyintersection(Vector<double>& ax, Vector<double>& ay, Vector<double>& bx, Vector<double>& by);

    static  Vector<double> quat_rot(Vector<double> point, Vector<double> rotVec, double angle);

    ///*************************************************************************************************************************
    // returns the value of x mapped through the sigmoid function
    // with parameters:
    // A the sigmoid center
    // B the sigmoid width
    ///*************************************************************************************************************************
    static double sigmoid(double x, double A, double B);






};

#endif // MYMATH_H
