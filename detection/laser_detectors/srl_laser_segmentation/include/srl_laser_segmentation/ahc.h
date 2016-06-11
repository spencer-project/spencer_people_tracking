#ifndef SRL_LASER_SEGMENTATION_AGGLOMERATIVE_HIERARCHICAL_H_
#define SRL_LASER_SEGMENTATION_AGGLOMERATIVE_HIERARCHICAL_H_

#include <srl_laser_segmentation/segmentation_algorithm.h>
#include <srl_laser_segmentation/efficient_ahc/efficient_ahc.h>


namespace srl_laser_segmentation {

/// @author Matthias Luber
class AgglomerativeHierarchicalClustering: public SegmentationAlgorithm, private EfficientAHC
{
public:
    /// Constructor
    AgglomerativeHierarchicalClustering(EfficientAHC::Linkage linkage, double distanceThreshold);

    /// Destructor
    virtual ~AgglomerativeHierarchicalClustering();

    /// Segment the given list of points. Consecutive points are assumed to be adjacent, i.e. the ordering of points is relevant.
    virtual void performSegmentation(const std::vector<Point2D>& points, std::vector<srl_laser_segmentation::LaserscanSegment::Ptr>& resultingSegments);
};

} // end of namespace srl_laser_segmentation

#endif // SRL_LASER_SEGMENTATION_AGGLOMERATIVE_HIERARCHICAL_H_
