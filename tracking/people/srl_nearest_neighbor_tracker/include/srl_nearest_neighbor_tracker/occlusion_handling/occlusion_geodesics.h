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

#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_HANDLING_OCCLUSION_GEODESICS_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_HANDLING_OCCLUSION_GEODESICS_H_
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <srl_nearest_neighbor_tracker/data/track.h>
#include <srl_nearest_neighbor_tracker/data/point_boost.h>
#include <srl_nearest_neighbor_tracker/data/polygon_boost.h>
#include <srl_nearest_neighbor_tracker/occlusion_handling/occlusion_data_utils.h>
#include <tf_conversions/tf_eigen.h>


#include <visualization_msgs/Marker.h>

#define foreach BOOST_FOREACH


namespace srl_nnt {


class OcclusionGeodesics{
public:
    OcclusionGeodesics(const Track::Ptr track,const double dimForward, const double dimBackward, const double dimWidth,const double res):
        m_centerX((double)track->state->x()(STATE_X_IDX)),
        m_centerY((double)track->state->x()(STATE_Y_IDX)),
        m_resolution(res),
        m_dimensionForward(dimForward),
        m_dimensionBackward(dimBackward),
        m_dimensionWidth(dimWidth),
        m_trackId(track->id),
        m_currenInfimumInitializer(0.0)
    {
        m_center = Point2D(m_centerX,m_centerY);
        double orientation = atan2((double)track->state->x()(STATE_VY_IDX),(double)track->state->x()(STATE_VX_IDX));
        m_orientation.setRPY(0,0,orientation);
        Eigen::Quaternion<double> prerotation (Eigen::AngleAxis<double>(M_PI,Eigen::Vector3d::UnitZ()));
        Eigen::Quaternion<double> rotation (Eigen::AngleAxis<double>(M_PI-orientation,Eigen::Vector3d::UnitZ()));

        Eigen::Translation<double,3> translation(Eigen::Vector3d(m_centerX,m_centerY,0));

        m_transform = rotation * translation * prerotation;
        m_transformInverse = m_transform.inverse();
        ROS_DEBUG_STREAM("Initializing occlusion geodesics for id " << m_trackId << " at center "<< m_center  << "with dimension forward"
                        << dimForward << "with dimension backward"
                        << dimBackward << "with dimension width"
                        << dimWidth<<  " and resolution " << res );
        m_numberCellsX = (unsigned int)(dimForward/res+dimBackward/res)+1;
        m_numberCellsY = (unsigned int) (dimWidth/res+1);
        m_numberCells = m_numberCellsX* m_numberCellsY;

        m_confidenceMap = ConfidenceMap::Zero(m_numberCellsY, m_numberCellsX);
        m_plausibilityConfidence = ConfidenceMap::Zero(m_numberCellsY, m_numberCellsX);
        m_inertialConfidence = ConfidenceMap::Zero(m_numberCellsY, m_numberCellsX);
        m_occlusionConfidence = ConfidenceMap::Zero(m_numberCellsY, m_numberCellsX);
        m_topLeft = Point2D(-((int)(dimBackward/res))*res,((int)(dimWidth/2.0/res))*res) ;
        for (size_t y = 0; y < m_numberCellsY; y++)
        {
            for (size_t x = 0; x < m_numberCellsX; x++)
            {
                Point2D point(m_topLeft.x+(x*m_resolution),m_topLeft.y-(y*m_resolution));
                m_gridPointsLocalFrame.push_back(point);
                m_gridPointsTrackerFrame.push_back(point.transform(m_transformInverse));
                ROS_DEBUG_STREAM("Added local point " << m_gridPointsLocalFrame.back() << " and tracker frame " << m_gridPointsTrackerFrame.back());
            }
        }

        m_outerLinestring.push_back(m_gridPointsTrackerFrame.front());
        m_outerLinestring.push_back(m_gridPointsTrackerFrame.at(m_numberCellsX-1));
        m_outerLinestring.push_back(m_gridPointsTrackerFrame.back());
        m_outerLinestring.push_back(*(m_gridPointsTrackerFrame.end()-m_numberCellsX));
        m_outerLinestring.push_back(m_gridPointsTrackerFrame.front());
        boost::geometry::correct(m_outerLinestring);


        ROS_DEBUG_STREAM("Initializing occlusion geodesics finished");
    }

    ~OcclusionGeodesics(){
        m_gridPointsLocalFrame.clear();
        m_gridPointsTrackerFrame.clear();
    }

    double getConfidenceAt(const ObsVector& observation)
    {
        Eigen::Vector3d observation3d;
        observation3d.head(OBS_DIM) = observation.head(OBS_DIM);
        observation3d(2) = 0;
        Eigen::Vector3d transformed = m_transform * observation3d;
        ROS_DEBUG_STREAM("Transformed observation:" << transformed);

        if (transformed(0) < -m_dimensionBackward || transformed(0) > m_dimensionForward)
            return std::numeric_limits<double>::infinity();
        if (transformed(1) < -m_dimensionWidth/2 || transformed(1) > m_dimensionWidth/2)
            return std::numeric_limits<double>::infinity();
        int idxX = round((double)(transformed(0) - m_topLeft.x)/m_resolution) ;
        int idxY = round((double)(transformed(1) + m_topLeft.y)/m_resolution) ;
        ROS_DEBUG_STREAM("Idx x " << idxX << " Idx y " << idxY);
        if ( idxX < 0) idxX = 0;
        if (idxX > m_numberCellsX-1 ) idxX = m_numberCellsX-1;
        if ( idxY < 0) idxY = 0;
        if (idxY > m_numberCellsY-1 ) idxY = m_numberCellsY-1;
        ROS_INFO_STREAM("Track "<< m_trackId <<" :Cost for (" << observation(0) << ";" << observation(1) << ") at indices "<< idxX <<";" << idxY << " is " << m_confidenceMap(idxY,idxX) << " for grid point " << m_gridPointsTrackerFrame.at(idxY*m_numberCellsX+idxX));
        return m_confidenceMap(idxY,idxX);
    }

    void updatePlausibilityConfidence(FilterState::Ptr begin, FilterState::Ptr current, unsigned int frame, const double motionVariance, const double plausibilityThreshold)
    {

        ROS_DEBUG_STREAM("updatePlausibilityConfidence");

        // Calculate plausibility term
        const double average_velocity = 1.5;
        const double average_distance_per_frame = 0.033;
        double stddev = min(sqrt(hypot((double)begin->C()(STATE_X_IDX, STATE_X_IDX),(double) begin->C()(STATE_Y_IDX, STATE_Y_IDX))), 1.0);

        double d_i_avg = max((average_distance_per_frame* frame) * (average_distance_per_frame* frame), stddev);
        Eigen::Vector2d d_i = begin->x().head(2)- current->xp().head(2);
        double d_i_norm= d_i.squaredNorm();
        double d_i_squared = std::max(d_i_norm,d_i_avg);
        double denominator = 2 * motionVariance * d_i_squared;

        Eigen::Vector2d beginPosition = begin->x().head(2);

        for (size_t i = 0; i < m_gridPointsTrackerFrame.size(); i++ )
        {
            caclulatePlausibilityConfidenceForCell(i, denominator, beginPosition, plausibilityThreshold);
        }
        ROS_DEBUG_STREAM("Updated plausibility confidence map to \n" << m_plausibilityConfidence);
    }

    void updateInertiaConfidence(FilterState::Ptr begin, FilterState::Ptr current, const double inertiaVariance)
    {
        ROS_DEBUG_STREAM("updateInertiaConfidence");

        const double average_distance_per_frame = 0.15;
        Eigen::Vector2d d_i = begin->x().head(2)- current->xp().head(2);
        Eigen::Vector2d beginPosition = begin->x().head(2);


        for (size_t i = 0; i < m_gridPointsTrackerFrame.size(); i++ )
        {
            caclulateInertiaConfidenceForCell(i, beginPosition, d_i, inertiaVariance);
        }
        ROS_DEBUG_STREAM("Updated inertial confidence map to \n" << m_inertialConfidence);

    }

    void updateOccludedConfidence(const OcclusionRegions& occlusionRegions,double detectorReliability, unsigned int frame)
    {
        ROS_DEBUG_STREAM("updateOccludedConfidence");
        // m_occlusionConfidence = ConfidenceMap::Constant(m_numberCellsY,m_numberCellsX, 1- std::pow(detectorReliability,frame));
        m_occlusionConfidence = ConfidenceMap::Constant(m_numberCellsY,m_numberCellsX, 1- std::pow(detectorReliability,1));
        foreach (OcclusionRegion::Ptr occlusionRegion, occlusionRegions){
            if (boost::geometry::intersects(m_outerLinestring, Linestring2D(occlusionRegion->occlusionPolygon.points.begin(), occlusionRegion->occlusionPolygon.points.end())))
            {
                for (size_t i = 0; i < m_gridPointsTrackerFrame.size(); i++ )
                {
                    calculateOcclusionConfidenceForCell(i,occlusionRegion->occlusionPolygon);
                }
            }
        }
        ROS_DEBUG_STREAM("Updated occlusion confidence map to \n" << m_occlusionConfidence << "\nwith detector reliability " << detectorReliability << " for frame " << frame);

    }

    void updateOcclusionCostMap(unsigned int cellRadius)
    {
        ROS_DEBUG_STREAM("updateOcclusionCostMap rows " << m_numberCellsY << " cols " << m_numberCellsX);
        ConfidenceMap minMap = ConfidenceMap::Zero(m_numberCellsY, m_numberCellsX);
        for (size_t row = 0; row < m_numberCellsY; row++)
        {
            int rowStart = row- cellRadius;
            int rowWidth = 2 * cellRadius +1;
            if (rowStart < 0){
                rowWidth += rowStart;
                rowStart = 0;
            }
            if (rowStart + rowWidth > m_numberCellsY-1){
                rowWidth += (m_numberCellsY-1) - (rowStart + rowWidth);
            }
            for (size_t col = 0; col < m_numberCellsX; col++)
            {
                int colStart = col - cellRadius;
                int colWidth = 2 * cellRadius +1;
                if (colStart < 0){
                    colWidth +=colStart;
                    colStart = 0;
                }
                if (colStart + colWidth > m_numberCellsX-1){
                    colWidth += (m_numberCellsX-1) - (colStart + colWidth);
                }
                ROS_DEBUG_STREAM("Looking for infima in submatrix start at " << rowStart << ";" << colStart << " and dimensions " << rowWidth << ";" << colWidth << " for position " << row << ";" << col << " size " <<m_confidenceMap.rows() << ";" << m_confidenceMap.cols() );

                Eigen::ArrayXXd subMatrix = m_confidenceMap.block(rowStart,colStart, rowWidth, colWidth);
                ROS_DEBUG_STREAM("Sub Array for infimum search \n" << subMatrix);
                double infimum =  subMatrix.minCoeff();
                ROS_DEBUG_STREAM("Infima in submatrix start at " << rowStart << ";" << colStart << " and dimensions " << rowWidth << ";" << colWidth << " is " << infimum);

                if (std::isnan(infimum) || std::isinf(infimum))
                    infimum = m_currenInfimumInitializer;
                minMap(row,col) = infimum;

            }
        }
        ROS_DEBUG_STREAM("Updated Infimum  map to \n" << minMap);

        m_confidenceMap = (ConfidenceMap::Ones(m_numberCellsY, m_numberCellsX) - (m_plausibilityConfidence * m_inertialConfidence *m_occlusionConfidence)) + minMap;
        ROS_DEBUG_STREAM("Updated confidence map to \n" << m_confidenceMap);

    }

    visualization_msgs::MarkerPtr getSelectedCellMarker(const ObsVector& observation, const std::string& frameID, const ros::Time time, const unsigned int id)
    {
        Eigen::Vector3d observation3d;
        observation3d.head(OBS_DIM) = observation.head(OBS_DIM);
        observation3d(2) = 0;
        Eigen::Vector3d transformed = m_transform * observation3d;
        ROS_DEBUG_STREAM("Transformed observation:" << transformed);

        if (transformed(0) < -m_dimensionBackward || transformed(0) > m_dimensionForward)
            return visualization_msgs::MarkerPtr();
        if (transformed(1) < -m_dimensionWidth/2 || transformed(1) > m_dimensionWidth/2)
            return visualization_msgs::MarkerPtr();
        int idxX = round((double)(transformed(0) - m_topLeft.x)/m_resolution) ;
        int idxY = round((double)(transformed(1) + m_topLeft.y)/m_resolution) ;
        if ( idxX < 0) idxX = 0;
        if (idxX > m_numberCellsX-1 ) idxX = m_numberCellsX-1;
        if ( idxY < 0) idxY = 0;
        if (idxY > m_numberCellsY-1 ) idxY = m_numberCellsY-1;

        visualization_msgs::MarkerPtr marker(new visualization_msgs::Marker);
        marker->action = visualization_msgs::Marker::ADD;
        marker->type = visualization_msgs::Marker::SPHERE;
        marker->ns = "occlusion_geodesics_tested_observations";
        marker->id = id;
        marker->header.frame_id = frameID;
        marker->header.stamp = time;
        marker->scale.x =marker->scale.y =marker->scale.z = m_resolution/2;
        marker->pose.orientation.w = 1.0;
        marker->pose.position.x = m_gridPointsTrackerFrame.at(idxY*m_numberCellsX+idxX).x;
        marker->pose.position.y = m_gridPointsTrackerFrame.at(idxY*m_numberCellsX+idxX).y;
        marker->pose.position.z = 0;
        marker->color.a = 0.5;
        marker->color.r = 1;
        marker->color.g = 1;
        marker->color.b = 1;


        return marker;
    }

    visualization_msgs::MarkerPtr getConfidenceMapMarker(const std::string& frameID, const ros::Time time, const unsigned int id, const unsigned int frame)
    {
        visualization_msgs::MarkerPtr marker(new visualization_msgs::Marker);
        marker->action = visualization_msgs::Marker::ADD;
        marker->type = visualization_msgs::Marker::CUBE_LIST;
        marker->ns = "occlusion_geodesics_confidence_map";
        marker->header.frame_id = frameID;
        marker->header.stamp = time;
        marker->id = id;
        marker->scale.x = m_resolution;
        marker->scale.y = m_resolution;
        marker->scale.z = 0;
        marker->pose.orientation.x = m_orientation.x();
        marker->pose.orientation.y = m_orientation.y();
        marker->pose.orientation.z = m_orientation.z();
        marker->pose.orientation.w = m_orientation.w();
        marker->pose.position.x = m_center.x;
        marker->pose.position.y = m_center.y;
        marker->pose.position.z = 0;
        double normalizer = 1;
        geometry_msgs::Point p;
        std_msgs::ColorRGBA color;
        color.a = 0.5;
        for (size_t i = 0; i < m_gridPointsLocalFrame.size(); i++ )
        {
            double cost = m_confidenceMap((int)i/m_numberCellsX,(int)i%m_numberCellsX);
            if (!(std::isnan(cost)|| std::isinf(cost)) && cost > normalizer)
                normalizer = cost;
        }

        for (size_t i = 0; i < m_gridPointsLocalFrame.size(); i++ )
        {
            p.x = m_gridPointsLocalFrame.at(i).x;
            p.y = m_gridPointsLocalFrame.at(i).y;
            marker->points.push_back(p);
            double cost = m_confidenceMap((int)i/m_numberCellsX,(int)i%m_numberCellsX)/normalizer;
            //            ROS_INFO_STREAM("Current cost for point " << i << "is " << cost << "map value " << m_confidenceMap((int)i/m_numberCells,(int)i%m_numberCells) << " normalizer " << normalizer);
            //            if (isnan(cost) || isinf(cost))
            //                continue;
            color.r = JetColors::red(cost);
            color.g = JetColors::green(cost);
            color.b = JetColors::blue(cost);
            marker->colors.push_back(color);
        }

        return marker;
    }

    visualization_msgs::MarkerPtr getPlausibilityConfidenceMapMarker(const std::string& frameID, const ros::Time time, const unsigned int id)
    {
        visualization_msgs::MarkerPtr marker(new visualization_msgs::Marker);
        marker->action = visualization_msgs::Marker::ADD;
        marker->type = visualization_msgs::Marker::CUBE_LIST;
        marker->ns = "occlusion_geodesics_plausibility_confidence_map";
        marker->header.frame_id = frameID;
        marker->header.stamp = time;
        marker->id = id;
        marker->scale.x = m_resolution;
        marker->scale.y = m_resolution;
        marker->scale.z = 0;
        marker->pose.orientation.x = m_orientation.x();
        marker->pose.orientation.y = m_orientation.y();
        marker->pose.orientation.z = m_orientation.z();
        marker->pose.orientation.w = m_orientation.w();
        marker->pose.position.x = m_center.x;
        marker->pose.position.y = m_center.y;
        marker->pose.position.z = 0;
        geometry_msgs::Point p;
        std_msgs::ColorRGBA color;
        color.a = 0.5;
        for (size_t i = 0; i < m_gridPointsLocalFrame.size(); i++ )
        {
            p.x = m_gridPointsLocalFrame.at(i).x;
            p.y = m_gridPointsLocalFrame.at(i).y;
            marker->points.push_back(p);
            double cost = m_plausibilityConfidence((int)i/m_numberCellsX,(int)i%m_numberCellsX);
            //            if (isnan(cost) || isinf(cost))
            //                continue;
            color.r = JetColors::red(cost);
            color.g = JetColors::green(cost);
            color.b = JetColors::blue(cost);
            marker->colors.push_back(color);
        }

        return marker;
    }

    visualization_msgs::MarkerPtr getInertialConfidenceMapMarker(const std::string& frameID, const ros::Time time, const unsigned int id)
    {
        visualization_msgs::MarkerPtr marker(new visualization_msgs::Marker);
        marker->action = visualization_msgs::Marker::ADD;
        marker->type = visualization_msgs::Marker::CUBE_LIST;
        marker->ns = "occlusion_geodesics_inertial_confidence_map";
        marker->header.frame_id = frameID;
        marker->header.stamp = time;
        marker->id = id;
        marker->scale.x = m_resolution;
        marker->scale.y = m_resolution;
        marker->scale.z = 0;
        marker->pose.orientation.x = m_orientation.x();
        marker->pose.orientation.y = m_orientation.y();
        marker->pose.orientation.z = m_orientation.z();
        marker->pose.orientation.w = m_orientation.w();
        marker->pose.position.x = m_center.x;
        marker->pose.position.y = m_center.y;
        marker->pose.position.z = 0;
        ROS_DEBUG_STREAM("FrameID of Marker is set to " << frameID);
        ROS_DEBUG_STREAM("Orientation of Marker is set to " << m_orientation.getX() << ";"<< m_orientation.getY()<< ";"<< m_orientation.getZ()<< ";"<< m_orientation.getW());
        geometry_msgs::Point p;
        std_msgs::ColorRGBA color;
        color.a = 0.5;
        for (size_t i = 0; i < m_gridPointsLocalFrame.size(); i++ )
        {
            p.x = m_gridPointsLocalFrame.at(i).x;
            p.y = m_gridPointsLocalFrame.at(i).y;
            marker->points.push_back(p);
            double cost = m_inertialConfidence((int)i/m_numberCellsX,(int)i%m_numberCellsX);
            color.r = JetColors::red(cost);
            color.g = JetColors::green(cost);
            color.b = JetColors::blue(cost);
            marker->colors.push_back(color);
        }
        return marker;
    }

    visualization_msgs::MarkerPtr getOcclusionConfidenceMapMarker(const std::string& frameID, const ros::Time time, const unsigned int id)
    {
        visualization_msgs::MarkerPtr marker(new visualization_msgs::Marker);
        marker->action = visualization_msgs::Marker::ADD;
        marker->type = visualization_msgs::Marker::CUBE_LIST;
        marker->ns = "occlusion_geodesics_occlusion_confidence_map";
        marker->header.frame_id = frameID;
        marker->header.stamp = time;
        marker->id = id;
        marker->scale.x = m_resolution;
        marker->scale.y = m_resolution;
        marker->scale.z = 0;
        marker->pose.orientation.x = m_orientation.x();
        marker->pose.orientation.y = m_orientation.y();
        marker->pose.orientation.z = m_orientation.z();
        marker->pose.orientation.w = m_orientation.w();
        marker->pose.position.x = m_center.x;
        marker->pose.position.y = m_center.y;
        marker->pose.position.z = 0;
        geometry_msgs::Point p;
        std_msgs::ColorRGBA color;
        color.a = 0.5;
        for (size_t i = 0; i < m_gridPointsLocalFrame.size(); i++ )
        {
            p.x = m_gridPointsLocalFrame.at(i).x;
            p.y = m_gridPointsLocalFrame.at(i).y;
            marker->points.push_back(p);
            double cost = m_occlusionConfidence((int)i/m_numberCellsX,(int)i%m_numberCellsX);
            color.r = JetColors::red(cost);
            color.g = JetColors::green(cost);
            color.b = JetColors::blue(cost);
            marker->colors.push_back(color);
        }

        return marker;
    }

    typedef boost::shared_ptr<OcclusionGeodesics> Ptr;
    typedef boost::shared_ptr<const OcclusionGeodesics> ConstPtr;


private:
    typedef Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> ConfidenceMap;

    ConfidenceMap m_confidenceMap;
    ConfidenceMap m_occlusionConfidence;
    ConfidenceMap m_plausibilityConfidence;
    ConfidenceMap m_inertialConfidence;
    double m_centerX;
    double m_centerY;
    Point2D m_center;
    Point2D m_topLeft;
    double m_dimensionForward;
    double m_dimensionBackward;
    double m_dimensionWidth;
    tf::Quaternion m_orientation;
    Eigen::Affine3d m_transform;
    Eigen::Affine3d m_transformInverse;
    track_id m_trackId;

    double m_resolution;
    unsigned int m_numberCells;
    unsigned int m_numberCellsX;
    unsigned int m_numberCellsY;
    std::vector<Point2D> m_gridPointsTrackerFrame;
    std::vector<Point2D> m_gridPointsLocalFrame;
    double m_currenInfimumInitializer;
    Linestring2D m_outerLinestring;

    inline void calculateOcclusionConfidenceForCell(unsigned int cellIdx, const Polygon2D& occlusionRegion)
    {
        if (boost::geometry::within(m_gridPointsTrackerFrame.at(cellIdx),occlusionRegion)){
            assert(cellIdx/m_numberCellsX < m_occlusionConfidence.rows());
            assert(cellIdx%m_numberCellsX < m_occlusionConfidence.cols());
            m_occlusionConfidence(cellIdx/m_numberCellsX,cellIdx%m_numberCellsX) = 1.0;
            ROS_DEBUG_STREAM("Point " << cellIdx << " is in polygon setting value to " <<  m_occlusionConfidence(cellIdx/m_numberCellsX,cellIdx%m_numberCellsX) );
        }
    }

    inline void caclulatePlausibilityConfidenceForCell(unsigned int cellIdx, const double denominator, const Eigen::Vector2d& beginPosition, const double cutOff)
    {
        Eigen::Vector2d location;
        location(0) = m_gridPointsTrackerFrame.at(cellIdx).x;
        location(1) = m_gridPointsTrackerFrame.at(cellIdx).y;
        Eigen::Vector2d d_j = beginPosition - location.head(2);
        double numerator = d_j.squaredNorm();
        double c_plausible = std::exp(-(numerator/denominator));
        if (c_plausible < cutOff){
            m_plausibilityConfidence(cellIdx/m_numberCellsX,cellIdx%m_numberCellsX) =-1* std::numeric_limits<double>::infinity();
        }
        else
            m_plausibilityConfidence(cellIdx/m_numberCellsX,cellIdx%m_numberCellsX) =  c_plausible;
    }

    inline void caclulateInertiaConfidenceForCell(unsigned int cellIdx, const Eigen::Vector2d& beginPosition, const Eigen::Vector2d& d_i, const double variance)
    {
        Eigen::Vector2d location;;
        location(0) = m_gridPointsTrackerFrame.at(cellIdx).x;
        location(1) = m_gridPointsTrackerFrame.at(cellIdx).y;
        Eigen::Vector2d d_j = beginPosition - location.head(2);

        // Calculate Inertia term
        double dot_result = d_i.dot(d_j);
        double di_dj_norm_product = d_i.norm() * d_j.norm();
        double numerator = (dot_result - di_dj_norm_product) * (dot_result - di_dj_norm_product);
        double denominator = 2* variance * d_i.squaredNorm() * d_j.squaredNorm();
        double c_inertia = std::exp(- numerator/denominator);
        if (std::isnan(c_inertia))
            c_inertia = 0.5;

        m_inertialConfidence(cellIdx/m_numberCellsX,cellIdx%m_numberCellsX) = c_inertia;

        double temp = m_confidenceMap(cellIdx/m_numberCellsX,cellIdx%m_numberCellsX);
        if (!(std::isnan(temp) || std::isinf(temp)) && temp > m_currenInfimumInitializer)
            m_currenInfimumInitializer =  m_confidenceMap(cellIdx/m_numberCellsX,cellIdx%m_numberCellsX);
    }

};


}  // namespace srl_nnt



#endif /* _SRL_NEAREST_NEIGHBOR_TRACKER_OCCLUSION_HANDLING_OCCLUSION_GEODESICS_H_ */
