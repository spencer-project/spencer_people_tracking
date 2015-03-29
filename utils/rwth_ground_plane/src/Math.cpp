#include "Math.h"

//*****************************************************************************
// Computes a singular value decomposition A=USV^T
// Input: U MxN matrix
// Output: U MxN matrix, S NxN diagonal matrix, V NxN diagonal matrix
// Credits: T. Brox
//******************************************************************************

void Math::svd(Matrix<double>& U, Matrix<double>& S, Matrix<double>& V, bool aOrdering, int aIterations) {
    static double at, bt, ct;
    static double maxarg1, maxarg2;
#define PYTHAG(a,b) ((at=fabs(a)) > (bt=fabs(b)) ?  (ct=bt/at,at*sqrt(1.0+ct*ct)) : (bt ? (ct=at/bt,bt*sqrt(1.0+ct*ct)): 0.0))
#define MAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ?	(maxarg1) : (maxarg2))
#define MIN(a,b) ((a) >(b) ? (b) : (a))
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
    int flag, i, its, j, jj, k, l, nm;
    double c, f, h, s, x, y, z;
    double anorm = 0.0, g = 0.0, scale = 0.0;
    int aXSize = U.x_size();
    int aYSize = U.y_size();
    Vector<double> aBuffer(aXSize);
    for (i = 0; i < aXSize; i++) {
        l = i + 1;
        aBuffer(i) = scale*g;
        g = s = scale = 0.0;
        if (i < aYSize) {
            for (k = i; k < aYSize; k++)
                scale += fabs(U(i, k));
            if (scale) {
                for (k = i; k < aYSize; k++) {
                    U(i, k) /= scale;
                    s += U(i, k) * U(i, k);
                }
                f = U(i, i);
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                U(i, i) = f - g;
                for (j = l; j < aXSize; j++) {
                    for (s = 0.0, k = i; k < aYSize; k++)
                        s += U(i, k) * U(j, k);
                    f = s / h;
                    for (k = i; k < aYSize; k++)
                        U(j, k) += f * U(i, k);
                }
                for (k = i; k < aYSize; k++)
                    U(i, k) *= scale;
            }
        }
        S(i, i) = scale*g;
        g = s = scale = 0.0;
        if (i < aYSize && i != aXSize - 1) {
            for (k = l; k < aXSize; k++)
                scale += fabs(U(k, i));
            if (scale != 0) {
                for (k = l; k < aXSize; k++) {
                    U(k, i) /= scale;
                    s += U(k, i) * U(k, i);
                }
                f = U(l, i);
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                U(l, i) = f - g;
                for (k = l; k < aXSize; k++)
                    aBuffer(k) = U(k, i) / h;
                for (j = l; j < aYSize; j++) {
                    for (s = 0.0, k = l; k < aXSize; k++)
                        s += U(k, j) * U(k, i);
                    for (k = l; k < aXSize; k++)
                        U(k, j) += s * aBuffer(k);
                }
                for (k = l; k < aXSize; k++)
                    U(k, i) *= scale;
            }
        }
        anorm = MAX(anorm, (fabs(S(i, i)) + fabs(aBuffer(i))));
    }
    for (i = aXSize - 1; i >= 0; i--) {
        if (i < aXSize - 1) {
            if (g != 0) {
                for (j = l; j < aXSize; j++)
                    V(i, j) = U(j, i) / (U(l, i) * g);
                for (j = l; j < aXSize; j++) {
                    for (s = 0.0, k = l; k < aXSize; k++)
                        s += U(k, i) * V(j, k);
                    for (k = l; k < aXSize; k++)
                        V(j, k) += s * V(i, k);
                }
            }
            for (j = l; j < aXSize; j++)
                V(j, i) = V(i, j) = 0.0;
        }
        V(i, i) = 1.0;
        g = aBuffer(i);
        l = i;
    }
    for (i = MIN(aYSize - 1, aXSize - 1); i >= 0; i--) {
        l = i + 1;
        g = S(i, i);
        for (j = l; j < aXSize; j++)
            U(j, i) = 0.0;
        if (g != 0) {
            g = 1.0 / g;
            for (j = l; j < aXSize; j++) {
                for (s = 0.0, k = l; k < aYSize; k++)
                    s += U(i, k) * U(j, k);
                f = (s / U(i, i)) * g;
                for (k = i; k < aYSize; k++)
                    U(j, k) += f * U(i, k);
            }
            for (j = i; j < aYSize; j++)
                U(i, j) *= g;
        } else {
            for (j = i; j < aYSize; j++)
                U(i, j) = 0.0;
        }
        ++U(i, i);
    }
    for (k = aXSize - 1; k >= 0; k--) {
        for (its = 1; its <= aIterations; its++) {
            flag = 1;
            for (l = k; l >= 0; l--) {
                nm = l - 1;
                if (fabs(aBuffer(l)) + anorm == anorm) {
                    flag = 0;
                    break;
                }
                if (fabs(S(nm, nm)) + anorm == anorm) break;
            }
            if (flag) {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++) {
                    f = s * aBuffer(i);
                    aBuffer(i) = c * aBuffer(i);
                    if (fabs(f) + anorm == anorm) break;
                    g = S(i, i);
                    h = PYTHAG(f, g);
                    S(i, i) = h;
                    h = 1.0 / h;
                    c = g*h;
                    s = -f*h;
                    for (j = 0; j < aYSize; j++) {
                        y = U(nm, j);
                        z = U(i, j);
                        U(nm, j) = y * c + z*s;
                        U(i, j) = z * c - y*s;
                    }
                }
            }
            z = S(k, k);
            if (l == k) {
                if (z < 0.0) {
                    S(k, k) = -z;
                    for (j = 0; j < aXSize; j++)
                        V(k, j) = -V(k, j);
                }
                break;
            }
            if (its == aIterations) std::cerr << "svd: No convergence in " << aIterations << " iterations" << std::endl;
            x = S(l, l);
            nm = k - 1;
            y = S(nm, nm);
            g = aBuffer(nm);
            h = aBuffer(k);
            f = ((y - z)*(y + z)+(g - h)*(g + h)) / (2.0 * h * y);
            g = PYTHAG(f, 1.0);
            f = ((x - z)*(x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
            c = s = 1.0;
            for (j = l; j <= nm; j++) {
                i = j + 1;
                g = aBuffer(i);
                y = S(i, i);
                h = s*g;
                g = c*g;
                z = PYTHAG(f, h);
                aBuffer(j) = z;
                double invZ = 1.0 / z;
                c = f*invZ;
                s = h*invZ;
                f = x * c + g*s;
                g = g * c - x*s;
                h = y*s;
                y *= c;
                for (jj = 0; jj < aXSize; jj++) {
                    x = V(j, jj);
                    z = V(i, jj);
                    V(j, jj) = x * c + z*s;
                    V(i, jj) = z * c - x*s;
                }
                z = PYTHAG(f, h);
                S(j, j) = z;
                if (z != 0) {
                    z = 1.0 / z;
                    c = f*z;
                    s = h*z;
                }
                f = (c * g)+(s * y);
                x = (c * y)-(s * g);
                for (jj = 0; jj < aYSize; jj++) {
                    y = U(j, jj);
                    z = U(i, jj);
                    U(j, jj) = y * c + z*s;
                    U(i, jj) = z * c - y*s;
                }
            }
            aBuffer(l) = 0.0;
            aBuffer(k) = f;
            S(k, k) = x;
        }
    }
    // Order singular values
    if (aOrdering) {
        for (int i = 0; i < aXSize - 1; i++) {
            int k = i;
            for (int j = i + 1; j < aXSize; j++)
                if (fabs(S(j, j)) > fabs(S(k, k))) k = j;
            if (k != i) {
                // Switch singular value i and k
                double help = S(k, k);
                S(k, k) = S(i, i);
                S(i, i) = help;
                // Switch columns i and k in U and V
                for (int j = 0; j < aYSize; j++) {
                    help = U(k, j);
                    U(k, j) = U(i, j);
                    U(i, j) = help;
                    help = V(k, j);
                    V(k, j) = V(i, j);
                    V(i, j) = help;
                }
            }
        }
    }
}

#undef PYTHAG
#undef MAX
#undef MIN
#undef SIGN

// Cyclic Jacobi method for determining the eigenvalues and eigenvectors
// of a symmetric matrix.
// Ref.:  H.R. Schwarz: Numerische Mathematik. Teubner, Stuttgart, 1988.
//        pp. 243-246.
// Credits: T. Brox

void Math::eigSymmMatrix(const Matrix<double>& aMatrix, Vector<double>& aEigenvalues, Matrix<double>& aEigenvectors) {
    static const double eps = 0.0001;
    static const double delta = 0.000001;
    static const double eps2 = eps*eps;
    double sum, theta, t, c, r, s, g, h;
    // Initialization
    Matrix<double> aCopy(aMatrix);
    int n = aEigenvalues.getSize();
    aEigenvectors = 0;
    for (int i = 0; i < n; i++)
        aEigenvectors(i, i) = 1;
    // Loop
    do {
        // check whether accuracy is reached
        sum = 0.0;
        for (int i = 1; i < n; i++)
            for (int j = 0; j <= i - 1; j++)
                sum += aCopy(i, j) * aCopy(i, j);
        if (sum + sum > eps2) {
            for (int p = 0; p < n - 1; p++)
                for (int q = p + 1; q < n; q++)
                    if (fabs(aCopy(q, p)) >= eps2) {
                        theta = (aCopy(q, q) - aCopy(p, p)) / (2.0 * aCopy(q, p));
                        t = 1.0;
                        if (fabs(theta) > delta) t = 1.0 / (theta + theta / fabs(theta) * sqrt(theta * theta + 1.0));
                        c = 1.0 / sqrt(1.0 + t * t);
                        s = c*t;
                        r = s / (1.0 + c);
                        aCopy(p, p) -= t * aCopy(q, p);
                        aCopy(q, q) += t * aCopy(q, p);
                        aCopy(q, p) = 0;
                        for (int j = 0; j <= p - 1; j++) {
                            g = aCopy(q, j) + r * aCopy(p, j);
                            h = aCopy(p, j) - r * aCopy(q, j);
                            aCopy(p, j) -= s*g;
                            aCopy(q, j) += s*h;
                        }
                        for (int i = p + 1; i <= q - 1; i++) {
                            g = aCopy(q, i) + r * aCopy(i, p);
                            h = aCopy(i, p) - r * aCopy(q, i);
                            aCopy(i, p) -= s * g;
                            aCopy(q, i) += s * h;
                        }
                        for (int i = q + 1; i < n; i++) {
                            g = aCopy(i, q) + r * aCopy(i, p);
                            h = aCopy(i, p) - r * aCopy(i, q);
                            aCopy(i, p) -= s * g;
                            aCopy(i, q) += s * h;
                        }
                        for (int i = 0; i < n; i++) {
                            g = aEigenvectors(i, q) + r * aEigenvectors(i, p);
                            h = aEigenvectors(i, p) - r * aEigenvectors(i, q);
                            aEigenvectors(i, p) -= s * g;
                            aEigenvectors(i, q) += s * h;
                        }
                    }
        }
    }            // Return eigenvalues
    while (sum + sum > eps2);
    for (int i = 0; i < n; i++)
        aEigenvalues(i) = aCopy(i, i);
    // Order eigenvalues and eigenvectors
    for (int i = 0; i < n - 1; i++) {
        int k = i;
        for (int j = i + 1; j < n; j++)
            if (fabs(aEigenvalues(j)) > fabs(aEigenvalues(k))) k = j;
        if (k != i) {
            // Switch eigenvalue i and k
            double help = aEigenvalues(k);
            aEigenvalues(k) = aEigenvalues(i);
            aEigenvalues(i) = help;
            // Switch eigenvector i and k
            for (int j = 0; j < n; j++) {
                help = aEigenvectors(j, k);
                aEigenvectors(j, k) = aEigenvectors(j, i);
                aEigenvectors(j, i) = help;
            }
        }
    }
}

bool Math::intersectRects(Vector<double> rect1, Vector<double> rect2) {
    //***************************************************************
    // Method for intersection of two axis aligned rectangles
    // rect1 / rect2 is a vector of 4 doubles in the following order
    // x-Position, y-Positon width and height.
    //***************************************************************

    assert(rect1.getSize() == 4 && rect2.getSize()== 4);

    if (rect2(0) < rect1(0) + rect1(2) &&
            rect2(1) < rect1(1) + rect1(3) &&
            rect2(0) + rect2(2) > rect1(0) &&
            rect2(1) + rect2(3) > rect1(1)) {
        return true;
    } else {
        return false;
    }
}


double Math::intersetRectSurface(Vector<double>& rect1, Vector<double>& rect2, Vector<double>& rectInter)
{
    //***************************************************************************
    // This method computes the intersction of two axis aligned rectangels
    // returns the surface and rectInter will contain the intersection rectangle
    //***************************************************************************
    assert(rect1.getSize() == 4 && rect2.getSize() == 4);

    double rect1_x = rect1(0);
    double rect1_y = rect1(1);
    double rect1_w = rect1(2);
    double rect1_h = rect1(3);

    double rect2_x = rect2(0);
    double rect2_y = rect2(1);
    double rect2_w = rect2(2);
    double rect2_h = rect2(3);

    double surface;

    double left = rect1_x > rect2_x ? rect1_x : rect2_x;
    double top = rect1_y > rect2_y ? rect1_y : rect2_y;
    double lhs = rect1_x + rect1_w;
    double rhs = rect2_x + rect2_w;
    double right = lhs < rhs ? lhs : rhs;
    lhs = rect1_y + rect1_h;
    rhs = rect2_y + rect2_h;
    double bottom = lhs < rhs ? lhs : rhs;

    rectInter.clearContent();
    rectInter.pushBack(right < left ? 0 : left);
    rectInter.pushBack(bottom < top ? 0 : top);
    rectInter.pushBack(right < left ? 0 : right - left);
    rectInter.pushBack(bottom < top ? 0 : bottom - top);

    surface = rectInter(2) * rectInter(3);
    return surface;
}

double Math::hist_bhatta(Volume<double> hist1, Volume<double> hist2) {

    //************************************************
    // Bhattacharyya distance of two histograms
    //************************************************
    double distance = 0;
    //        #pragma omp parallel for reduction(+ : distance)
    for(int i = 0; i < hist1.xSize(); i++)
    {
        for(int j = 0; j < hist1.ySize(); j++)
        {
            for(int l = 0; l < hist1.zSize(); l++)
            {
                distance += sqrtf(hist1(i,j,l)*hist2(i,j,l));
            }
        }
    }
    return distance;
}

double Math::hist_bhatta(double* hist1, double* hist2, int size) {

    //************************************************
    // Bhattacharyya distance of two histograms
    //************************************************
    double distance = 0;
    //        #pragma omp parallel for reduction(+ : distance)
    for(int i = 0; i < size; i++)
    {
        distance += sqrtf(hist1[i]*hist2[i]);
    }
    return distance;
}

void Math::PATransformation(const Matrix<double>& aMatrix, Vector<double>& aEigenvalues, Matrix<double>& aEigenvectors) {

    // PATransformation
    // Cyclic Jacobi method for determining the eigenvalues and eigenvectors
    // of a symmetric matrix.
    // Ref.:  H.R. Schwarz: Numerische Mathematik. Teubner, Stuttgart, 1988.
    //        pp. 243-246.
    // Credits: T. Brox

    static const double eps = 0.0001;
    static const double delta = 0.000001;
    static const double eps2 = eps*eps;
    double sum,theta,t,c,r,s,g,h;
    // Initialization
    Matrix<double> aCopy(aMatrix);
    int n = aEigenvalues.getSize();
    aEigenvectors = 0;
    for (int i = 0; i < n; i++)
        aEigenvectors(i,i) = 1;
    // Loop
    do {
        // check whether accuracy is reached
        sum = 0.0;
        for (int i = 1; i < n; i++)
            for (int j = 0; j <= i-1; j++)
                sum += aCopy(i,j)*aCopy(i,j);
        if (sum+sum > eps2) {
            for (int p = 0; p < n-1; p++)
                for (int q = p+1; q < n; q++)
                    if (fabs(aCopy(q,p)) >= eps2) {
                        theta = (aCopy(q,q) - aCopy(p,p)) / (2.0 * aCopy(q,p));
                        t = 1.0;
                        if (fabs(theta) > delta) t = 1.0 / (theta + theta/fabs(theta) * sqrt (theta*theta + 1.0));
                        c = 1.0 / sqrt (1.0 + t*t);
                        s = c*t;
                        r = s / (1.0 + c);
                        aCopy(p,p) -= t * aCopy(q,p);
                        aCopy(q,q) += t * aCopy(q,p);
                        aCopy(q,p) = 0;
                        for (int j = 0; j <= p-1; j++) {
                            g = aCopy(q,j) + r * aCopy(p,j);
                            h = aCopy(p,j) - r * aCopy(q,j);
                            aCopy(p,j) -= s*g;
                            aCopy(q,j) += s*h;
                        }
                        for (int i = p+1; i <= q-1; i++) {
                            g = aCopy(q,i) + r * aCopy(i,p);
                            h = aCopy(i,p) - r * aCopy(q,i);
                            aCopy(i,p) -= s * g;
                            aCopy(q,i) += s * h;
                        }
                        for (int i = q+1; i < n; i++) {
                            g = aCopy(i,q) + r * aCopy(i,p);
                            h = aCopy(i,p) - r * aCopy(i,q);
                            aCopy(i,p) -= s * g;
                            aCopy(i,q) += s * h;
                        }
                        for (int i = 0; i < n; i++) {
                            g = aEigenvectors(i,q) + r * aEigenvectors(i,p);
                            h = aEigenvectors(i,p) - r * aEigenvectors(i,q);
                            aEigenvectors(i,p) -= s * g;
                            aEigenvectors(i,q) += s * h;
                        }
                    }
        }
    }
    // Return eigenvalues
    while (sum+sum > eps2);
    for (int i = 0; i < n; i++)
        aEigenvalues(i) = aCopy(i,i);
    // Order eigenvalues and eigenvectors
    //        for (int i = 0; i < n-1; i++) {
    //          int k = i;
    //          for (int j = i+1; j < n; j++)
    //            if (fabs(aEigenvalues(j)) > fabs(aEigenvalues(k))) k = j;
    //          if (k != i) {
    //            // Switch eigenvalue i and k
    //            double help = aEigenvalues(k);
    //            aEigenvalues(k) = aEigenvalues(i);
    //            aEigenvalues(i) = help;
    //            // Switch eigenvector i and k
    //            for (int j = 0; j < n; j++) {
    //              help = aEigenvectors(j,k);
    //              aEigenvectors(j,k) = aEigenvectors(j,i);
    //              aEigenvectors(j,i) = help;
    //            }
    //          }
    //        }
}


void Math::PABacktransformation(const Matrix<double>& aEigenvectors, const Vector<double>& aEigenvalues, Matrix<double>& aMatrix) {

    // PABackTransformation

    for (int i = 0; i < aEigenvalues.getSize(); i++)
        for (int j = 0; j <= i; j++) {
            double sum = aEigenvalues(0) * aEigenvectors(i,0) * aEigenvectors(j,0);
            for (int k = 1; k < aEigenvalues.getSize(); k++)
                sum += aEigenvalues(k) * aEigenvectors(i,k) * aEigenvectors(j,k);
            aMatrix(i,j) = sum;
        }
    for (int i = 0; i < aEigenvalues.getSize(); i++)
        for (int j = i+1; j < aEigenvalues.getSize(); j++)
            aMatrix(i,j) = aMatrix(j,i);
}

bool Math::evalElipse(double w, double h, double centerX, double centerY, double x, double y)
{
    // Evaluate if a point is inside an elipse
    if((((x-centerX)*2.0 / w)*((x-centerX)*2.0 / w) + ((y-centerY)*2.0 / h)*((y-centerY)*2.0 / h)) <= 1)
        return true;

    return false;
}

double Math::evalElipseV(double h, double w, double centerX, double centerY, double x, double y)
{
    return (((x-centerX)*2.0 / w)*((x-centerX)*2.0 / w) + ((y-centerY)*2.0 / h)*((y-centerY)*2.0 / h));
}

void Math::crossLine(Vector<double> o1, Vector<double> r1, Vector<double> o2, Vector<double> r2, Vector<double>& crossPoint)
{

    Vector<double> copyo1;
    Vector<double> copyo2;
    Vector<double> copyr1;
    Vector<double> copyr2;

    copyo1 = o1;
    copyo2 = o2;
    copyr1 = r1;
    copyr2 = r2;

    copyo2 -= copyo1;
    copyr2 *= -1.0;
    double D_s = copyo2(0)*copyr2(1) - copyo2(1)*copyr2(0);
    double D   = r1(0)*copyr2(1) - r1(1)*copyr2(0);
    double s   = D_s / D;

    crossPoint.setSize(o1.getSize(), 0.0);
    copyr1 = r1;
    copyr1 *= s;
    crossPoint += o1;
    crossPoint += copyr1;

}

//**********************************************************
// Author Andreas Ess
//**********************************************************

double Math::polyintersection(Vector<double>& ax, Vector<double>& ay, Vector<double>& bx, Vector<double>& by)
{


    double result;

    int adim = 4;
    int bdim = 4;


    // Create polygons
    vector< Point2D > poly[2], v;
    for (int i = 0; i < adim; i++)
        poly[0].push_back(Point2D(ax(i), ay(i)));
    for (int i = 0; i < bdim; i++)
        poly[1].push_back(Point2D(bx(i), by(i)));

    // Add all inlying points
    for (int i = 0; i < adim; i++) {
        if (inside(poly[1], poly[0][i]))
            v.push_back(poly[0][i]);
    }
    for (int i = 0; i < bdim; i++) {
        if (inside(poly[0], poly[1][i]))
            v.push_back(poly[1][i]);
    }

    // Add intersections
    for (int i = 0; i < adim; i++) {
        for (int j = 0; j < bdim; j++) {
            Point2D c = intersect(poly[0][i], poly[0][(i+1)%adim], poly[1][j], poly[1][(j+1)%bdim]);
            if (c.x != 1e8)
                v.push_back(c);
        }
    }



    if (v.size() < 3) {
        result = 0.0;
    }else
    {
        convexHull(v);
        result = fabs(area(v));
    }

    return result;
}



double Math::dot(const Point2D& a, const Point2D& b) {
    return a.x * b.x + a.y * b.y;
}



bool vector_sort(const Point2D& v1, const Point2D& v2)
{
    double r = v1.det(v2);
    if (fabs(r) > 0.000001) return r < 0;
    return v1.normSq() - v2.normSq() < 0;
}

// Calculate the convex hull of a polygon (or just a set of points)
// The coordinates in the convex hull will be in clockwise direction.
void Math::convexHull(vector< Point2D >& v)
{
    int i, j, m, n = (int)v.size();
    vector< Point2D > hull;

    hull.resize(n);
    for(j = 0, i = 1; i < n; i++)
        if ((v[i].x < v[j].x) || (v[i].x == v[j].x && v[i].y < v[j].y))
            j = i;

    for(i = 0; i < n; i++)
        hull[i] = v[i] - v[j];

    if (hull.size() > 0)
        sort(hull.begin(), hull.end(), vector_sort);

    for(i = m = 0; i < n; i++) {
        hull[i] = hull[i] + v[j];
        if (m && hull[i] == hull[m - 1]) continue;

        while (m > 1 && hull[m - 2].cross(hull[m - 1], hull[i]) >= 0)
            m--;

        hull[m++] = hull[i];
    }

    for (int i = 0; i < m; i++)
        v[i] = hull[i];
    v.resize(m);
}

double Math::area(const vector< Point2D >& points) {
    double ret = 0;
    for (unsigned int pIdx = 0; pIdx < points.size(); pIdx++) {
        unsigned int pIdx2 = (pIdx + 1) % points.size();
        ret += points[pIdx].x * points[pIdx2].y - points[pIdx].y * points[pIdx2].x;
    }

    return 0.5 * ret;
}

/**
 * Calculate line in general form given two points
 **/
void Math::calcLine(const Point2D& a1, const Point2D& a2, HomgPoint2D& l) {
    double x1 = a1.x;
    double x2 = a2.x;
    double y1 = a1.y;
    double y2 = a2.y;

    double norm = sqrt((y1-y2)*(y1-y2) + (x2-x1)*(x2-x1));
    l.x = (y1 - y2) / norm;
    l.y = (x2 - x1) / norm;
    l.w = (x1 * y2 - x2 * y1) / norm;
}

/**
 * Find intersection point of two lines in general form
 * Returns nan if no intersection
 **/
Point2D Math::intersect(const Point2D& a1, const Point2D& a2, const Point2D& b1, const Point2D &b2) {
    Point2D res;
    res.x = 1e8;

    HomgPoint2D l1, l2;
    calcLine(a1, a2, l1);
    calcLine(b1, b2, l2);

    if (l1 == l2) {
        // Special case of collinearity

    } else {
        // Intersect
        HomgPoint2D c = l1.cross(l2);
        Point2D cc(c.x / c.w, c.y / c.w);

        Point2D v = a2 - a1;
        double t1 = dot(v, cc - a1) / v.normSq();
        v = b2 - b1;
        double t2 = dot(v, cc - b1) / v.normSq();

        if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1)
            res = cc;
    }

    return res;
}

bool Math::inside(const vector< Point2D >& v, const Point2D& pt) {
    if (v.size() < 3)
        return false;

    vector< Point2D >::const_iterator it;
    bool side = v.front().cross(v.back(), pt) < 0;
    for (it = v.begin(); it != v.end() - 1; it++) {
        bool newside = (it + 1)->cross(*it, pt) < 0;
        if (newside != side)
            return false;
    }
    return true;
}

void Math::invRegularized(Matrix<double>& src, double aMaxValue,double aMinValue)
{
    assert(src.x_size() == src.y_size());

    Vector<double> eVals(src.x_size() );
    Matrix<double> eVecs(src.x_size(), src.x_size());

    PATransformation(src,eVals,eVecs);

    for (int i = 0 ; i < src.x_size() ; i++)
    {
        //            double lowLevel = 1.0/(aMinValue*aMinValue);
        //            double highLevel = 1.0/(aMaxValue*aMaxValue);
        double lowLevel = (aMaxValue*aMaxValue);
        double highLevel = (aMinValue*aMinValue);

        //            double lowLevel = 1.0/aMinValue;
        //            double highLevel = 1.0/MaxValue;

        if (eVals(i) > lowLevel)
        {
            eVals(i) = lowLevel;
        }

        else if(eVals(i) < highLevel)
        {
            eVals(i) = highLevel;
        }
        //            else
        //            {
        //                eVals(i) = eVals(i);
        //            }
    }
    PABacktransformation(eVecs,eVals,src);
}

Vector<double> Math::quat_rot(Vector<double> point, Vector<double> rotVec, double angle)
{
    // normalize rotVec first
    rotVec *= 1.0/rotVec.norm();

    Vector<double> result;
    Point p(point(0), point(1), point(2));
    Point v(rotVec(0), rotVec(1), rotVec(2));

    Quaternion q;
    Point r = q.quat_rot(p, v, angle);

    result.pushBack(r.x);
    result.pushBack(r.y);
    result.pushBack(r.z);

    return result;
}

double Math::sigmoid(double x, double A, double B)
{
    return 1.0/(1.0+exp(-B*(x-A)));
}
