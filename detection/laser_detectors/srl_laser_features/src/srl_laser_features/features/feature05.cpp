#include <srl_laser_features/features/feature05.h>
#include <float.h>

namespace srl_laser_features
{

Feature05::Feature05(bool extended) : Feature(), m_extended(extended)
{
}

void Feature05::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
    result = Eigen::VectorXd::Zero(getNDimensions());

    const size_t numPoints = segment.points.size();
    if (numPoints > 1) {
        // width
        result(0) = (segment.points.back() - segment.points.front()).norm();

        if (m_extended) {
            // min and max distance between points
            result(1) = DBL_MAX;
            result(2) = 0.0;

            for (size_t pIndex = 0; pIndex < numPoints - 1; ++pIndex) {
                double dist = (segment.points[pIndex] - segment.points[pIndex + 1]).norm();
                if (dist < result(1)) result(1) = dist;
                if (dist > result(2)) result(2) = dist;
            }

            // ratio between min and max distance between points
            if (result(2) != 0.0) {
                result(3) = result(1) / result(2);
            }
        }
    }
    else {
        result.setConstant(getNDimensions(), -1.0);
    }
}

} // end of namespace srl_laser_features
