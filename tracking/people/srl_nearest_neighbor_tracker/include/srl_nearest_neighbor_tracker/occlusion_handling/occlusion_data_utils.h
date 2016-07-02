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

#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_HANDLING_OCCLUSION_DATA_UTILS_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_HANDLING_OCCLUSION_DATA_UTILS_H_

#include <boost/shared_ptr.hpp>
#include <srl_nearest_neighbor_tracker/data/point_boost.h>
#include <srl_nearest_neighbor_tracker/data/polygon_boost.h>
#include <srl_nearest_neighbor_tracker/data/track.h>
#include <ros/time.h>


namespace srl_nnt {


/// struct for saving interesting data for occlusion handling for segments
struct SegmentInfo{
    double minAngle;
    double maxAngle;
    double distAtMinAngle;
    double distAtMaxAngle;
    unsigned int label;
    unsigned int idxSegmentation;
    /// for easier readability
    typedef boost::shared_ptr<SegmentInfo> Ptr;
};
typedef std::vector<SegmentInfo::Ptr> SegmentInfoList;
typedef std::vector<Polygon2D> PolygonList;

struct OcclusionRegion{
    Polygon2D occlusionPolygon;
    Polygon2DList occlusionRiskPolygons;
    SegmentInfoList responsableSegments;
    typedef boost::shared_ptr<OcclusionRegion> Ptr;
};

typedef std::vector<OcclusionRegion::Ptr> OcclusionRegions;

class JetColors{
public:
    static double red( double gray ) {
        return base( gray - 0.25 );
    }
    static double green( double gray ) {
        return base( gray );
    }
    static double blue( double gray ) {
        return base( gray + 0.25 );
    }

private:

    static double interpolate( double val, double y0, double x0, double y1, double x1 ) {
        return (val-x0)*(y1-y0)/(x1-x0) + y0;
    }

    static double base( double val ) {
        if ( val <= 0.125 ) return 0;
        else if ( val <= 0.375 ) return interpolate( val, 0.0, 0.125, 1.0, 0.375 );
        else if ( val <= 0.625 ) return 1.0;
        else if ( val <= 0.875 ) return interpolate( val, 1.0, 0.625, 0.0, 0.875 );
        else return 0.0;
    }

    /// Singleton instance of this class
    static JetColors* s_instance;

};






}  // namespace srl_nnt



#endif /* _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_HANDLING_OCCLUSION_DATA_UTILS_H_ */
