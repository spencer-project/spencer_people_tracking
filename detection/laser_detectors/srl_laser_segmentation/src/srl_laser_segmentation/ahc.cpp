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