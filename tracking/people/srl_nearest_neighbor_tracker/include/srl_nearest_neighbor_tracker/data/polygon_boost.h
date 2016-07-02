/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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

#ifndef _POLYGON_BOOST_H
#define _POLYGON_BOOST_H
#include <srl_nearest_neighbor_tracker/data/point_boost.h>

#include <boost/assert.hpp>

#include <boost/iterator.hpp>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/iterator/iterator_categories.hpp>
#include <boost/iterator/iterator_facade.hpp>


#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/util/add_const_if_c.hpp>

#include <visualization_msgs/Marker.h>


// Sample polygon, having legacy methods
// (similar to e.g. COM objects)
class Polygon2D
{
public :
    std::vector<Point2D> points;

    void add_point(Point2D const& p) { points.push_back(p); }

    // Const access
    Point2D const& get_point(std::size_t i) const
    {
        BOOST_ASSERT(i < points.size());
        return points[i];
    }

    // Mutable access
    Point2D & get_point(std::size_t i)
    {
        BOOST_ASSERT(i < points.size());
        return points[i];
    }

    void get_points_for_marker(visualization_msgs::Marker& marker, double z=0.0)
    {
        marker.points.clear();
        geometry_msgs::Point p;
        for(std::size_t i=0 ; i < points.size(); i++ )
        {
            p.x = points.at(i).x;
            p.y = points.at(i).y;
            p.z = z;
            marker.points.push_back(p);
        }
    }



    int point_count() const { return points.size(); }
    void erase_all() { points.clear(); }

    inline void set_size(int n) { points.resize(n); }
};

typedef std::vector<Polygon2D> Polygon2DList;




// ----------------------------------------------------------------------------
// Adaption: implement iterator and range-extension, and register with Boost.Geometry

// 1) implement iterator (const and non-const versions)
template<typename MyPolygon>
struct custom_iterator : public boost::iterator_facade
<
custom_iterator<MyPolygon>,
Point2D,
boost::random_access_traversal_tag,
typename boost::mpl::if_
<
boost::is_const<MyPolygon>,
Point2D const,
Point2D
>::type&
>
{
    // Constructor for begin()
    explicit custom_iterator(MyPolygon& polygon)
    : m_polygon(&polygon)
    , m_index(0)
    {}

    // Constructor for end()
    explicit custom_iterator(bool, MyPolygon& polygon)
    : m_polygon(&polygon)
    , m_index(polygon.point_count())
    {}


    // Default constructor
    explicit custom_iterator()
    : m_polygon(NULL)
    , m_index(-1)
    {}

    typedef typename boost::mpl::if_
            <
            boost::is_const<MyPolygon>,
            Point2D const,
            Point2D
            >::type Point2D_type;

private:
    friend class boost::iterator_core_access;


    typedef boost::iterator_facade
            <
            custom_iterator<MyPolygon>,
            Point2D,
            boost::random_access_traversal_tag,
            Point2D_type&
            > facade;

    MyPolygon* m_polygon;
    int m_index;

    bool equal(custom_iterator const& other) const
    {
        return this->m_index == other.m_index;
    }
    typename facade::difference_type distance_to(custom_iterator const& other) const
    {
        return other.m_index - this->m_index;
    }

    void advance(typename facade::difference_type n)
    {
        m_index += n;
        if(m_polygon != NULL
                && (m_index >= m_polygon->point_count()
                        || m_index < 0)
        )
        {
            m_index = m_polygon->point_count();
        }
    }

    void increment()
    {
        advance(1);
    }

    void decrement()
    {
        advance(-1);
    }

    // const and non-const dereference of this iterator
    Point2D_type& dereference() const
    {
        return m_polygon->get_point(m_index);
    }
};




// 2) Implement Boost.Range const functionality
//    using method 2, "provide free-standing functions and specialize metafunctions"
// 2a) meta-functions
namespace boost
{
template<> struct range_mutable_iterator<Polygon2D>
{
    typedef custom_iterator<Polygon2D> type;
};

template<> struct range_const_iterator<Polygon2D>
{
    typedef custom_iterator<Polygon2D const> type;
};

// RangeEx
template<> struct range_size<Polygon2D>
{
    typedef std::size_t type;
};

} // namespace 'boost'


// 2b) free-standing function for Boost.Range ADP
inline custom_iterator<Polygon2D> range_begin(Polygon2D& polygon)
        {
    return custom_iterator<Polygon2D>(polygon);
        }

inline custom_iterator<Polygon2D const> range_begin(Polygon2D const& polygon)
        {
    return custom_iterator<Polygon2D const>(polygon);
        }

inline custom_iterator<Polygon2D> range_end(Polygon2D& polygon)
        {
    return custom_iterator<Polygon2D>(true, polygon);
        }

inline custom_iterator<Polygon2D const> range_end(Polygon2D const& polygon)
        {
    return custom_iterator<Polygon2D const>(true, polygon);
        }



// 3) optional, for writable geometries only, implement push_back/resize/clear
namespace boost { namespace geometry { namespace traits
{

template<> struct push_back<Polygon2D>
{
    static inline void apply(Polygon2D& polygon, Point2D const& point)
    {
        polygon.add_point(point);
    }
};

template<> struct resize<Polygon2D>
{
    static inline void apply(Polygon2D& polygon, std::size_t new_size)
    {
        polygon.set_size(new_size);
    }
};

template<> struct clear<Polygon2D>
{
    static inline void apply(Polygon2D& polygon)
    {
        polygon.erase_all();
    }
};

}}}


// 4) register with Boost.Geometry
//BOOST_GEOMETRY_REGISTER_POINT_2D(Point2D, double, cs::cartesian, x, y)

BOOST_GEOMETRY_REGISTER_RING(Polygon2D)

#endif //_POLYGON_BOOST_H
