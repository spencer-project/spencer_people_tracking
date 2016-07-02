/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2016, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _POINT_BOOST_H
#define _POINT_BOOST_H

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <math.h>       /* hypot */
#include <ostream>




// Sample point, having x/y
class Point2D
{
public:
    Point2D(double a = 0, double b = 0) : x(a), y(b) {}
    double x,y;

    Point2D transform(const Eigen::Affine3d& transform)
    {
        Eigen::Vector3d vector;
        vector(0) = this->x;
        vector(1) = this->y;
        vector(2) = 0;
        Eigen::Vector3d transformed = transform * vector;
        return Point2D(transformed(0), transformed(1));
    }

    Point2D operator+(const Point2D& p)
    {
        Point2D result;
        result.x = this->x + p.x;
        result.y = this->y + p.y;
        return result;
    }

    Point2D operator-(const Point2D& p)
    {
        Point2D result;
        result.x = this->x - p.x;
        result.y = this->y - p.y;
        return result;
    }


    Point2D operator*(const double factor)
    {
        Point2D result;
        result.x = this->x * factor;
        result.y = this->y * factor;
        return result;
    }

    Point2D operator/(const double factor)
    {
        Point2D result;
        result.x = this->x / factor;
        result.y = this->y / factor;
        return result;
    }

    double distance(const Point2D& p)
    {
        return hypot(this->x - p.x, this->y - p.y);
    }

    double distanceSquared(const Point2D& p)
    {
        return (this->x - p.x)*(this->x - p.x)+ (this->y - p.y)*(this->y - p.y);
    }

    friend std::ostream& operator<< (std::ostream &out, const Point2D& p)
    {
        // Since operator<< is a friend of the Point class, we can access
        // Point's members directly.
        out << "(X:" << p.x << ";Y:" << p.y << ")";
        return out;
    }

};


// 4) register with Boost.Geometry
BOOST_GEOMETRY_REGISTER_POINT_2D(Point2D, double, cs::cartesian, x, y)

#endif
