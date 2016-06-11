/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  Copyright (c) 2006-2012, Matthias Luber, Social Robotics Lab, University of Freiburg
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

#include <srl_laser_segmentation/ahc.h>
#include <ros/console.h>
#include <exception>

namespace srl_laser_segmentation {

AgglomerativeHierarchicalClustering::AgglomerativeHierarchicalClustering(EfficientAHC::Linkage linkage, double distanceThreshold) : EfficientAHC(linkage, distanceThreshold)
{
    ROS_INFO("Initializing agglomerative hierarchical clustering segmentation with %s linkage!", LinkageString[linkage].c_str());
}


AgglomerativeHierarchicalClustering::~AgglomerativeHierarchicalClustering()
{
}


void AgglomerativeHierarchicalClustering::performSegmentation(const std::vector<Point2D>& points, std::vector<srl_laser_segmentation::LaserscanSegment::Ptr>& resultingSegments)
{
    ROS_DEBUG_NAMED("AgglomerativeHierarchicalClustering", "AgglomerativeHierarchicalClustering::%s", __func__);

    // Filter out invalid measurements
    typedef unsigned int point_index;
    std::vector<Point2D> filteredPoints;
    std::map<point_index, point_index> pointMapping;

    for(size_t i = 0; i < points.size(); i++) {
        if(isValidMeasurement(&points[i])) {
            pointMapping[ filteredPoints.size() ] = i;
            filteredPoints.push_back(points[i]);
        }
    }

    // Initialize linkage
    switch (m_linkage)
    {
    case SINGLE:
        initializeSingle(filteredPoints);
        break;

    case AVERAGE_CPU:
        initializeAverage(filteredPoints);
        break;

    case COMPLETE:
        initializeComplete(filteredPoints);
        break;

    case AVERAGE_MEM:
        // Not implemented
        throw std::exception();
    }

    // Build dendogram
    Eigen::MatrixXd unusedMatrix;
    buildDendrogram(unusedMatrix);

    // Threshold dendrogram and extract segments
    std::vector<DendroNode*> nodes;
    for (unsigned int dIndex = 0; dIndex < N; ++dIndex) {
        if (I[dIndex]) {
            nodes.push_back(m_rootNodes[dIndex]);
        }
    }

    unsigned int segmentCounter = 0;
    while (!nodes.empty()) {
        DendroNode* node = nodes.back();
        nodes.pop_back();

        if (node->getDistance() > m_threshold) {
            nodes.push_back(node->getLeft());
            nodes.push_back(node->getRight());
        }
        else {
            // Got a new segment
            srl_laser_segmentation::LaserscanSegment::Ptr segment(new srl_laser_segmentation::LaserscanSegment);
            segment->label = segmentCounter++;
            for(size_t i = 0; i < node->getPoints().size(); i++) {
                point_index filteredPointIndex = node->getPoints()[i];
                point_index originalPointIndex = pointMapping[ filteredPointIndex ];
                segment->measurement_indices.push_back( originalPointIndex );
            }
            resultingSegments.push_back(segment);
        }
    }

    // clean up dendrogram
    for (unsigned int dIndex = 0; dIndex < m_dendroNodes.size(); ++dIndex) {
        delete m_dendroNodes[dIndex];
    }
    m_dendroNodes.clear();
}


} // end of namespace srl_laser_segmentation
