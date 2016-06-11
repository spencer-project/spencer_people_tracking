/*
* Software License Agreement (BSD License)
*
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

#include <srl_laser_segmentation/efficient_ahc/efficient_ahc.h>
#include <ros/console.h>
#include <exception>

#define DELETE_KEY INT_MIN // this key MUST not be allowed as regular key!


namespace srl_laser_segmentation {

const int EfficientAHC::DOUBLE_TO_INT_MULTIPLIER = 1000000;


DendroNode::DendroNode(unsigned int index) : m_left(NULL), m_right(NULL), m_distance(0.0)
{
    m_points.push_back(index);
}

DendroNode::DendroNode(const std::vector<unsigned int>& indices) : m_left(NULL), m_right(NULL), m_distance(0.0)
{
    m_points.insert(m_points.end(), indices.begin(), indices.end());
}

DendroNode::DendroNode(DendroNode* left, DendroNode* right, double distance) : m_left(left), m_right(right), m_distance(distance)
{
    m_points.insert(m_points.end(), m_left->m_points.begin(), m_left->m_points.end());
    m_points.insert(m_points.end(), m_right->m_points.begin(), m_right->m_points.end());
}


EfficientAHC::EfficientAHC(Linkage linkage, double threshold)
    : m_linkage(linkage), m_threshold(threshold), m_allocatedN(0), N(0), CC(NULL), m_dataObjectRequired(false), I(NULL), P(NULL), m_rootNodes(NULL), m_createFunction(0), m_initFunction(0), m_deleteFunction(0), m_updateFunction(0)
{
    ROS_DEBUG_NAMED("EfficientAHC", "EfficientAHC( %i )", linkage);

    // link to default functions
    switch (m_linkage)
    {
    default:
    case SINGLE:
        m_dataObjectRequired = false;
        m_updateFunction = *updateSingle;
        break;

    case AVERAGE_CPU:
        m_dataObjectRequired = true;
        m_createFunction = *createAverageCPU;
        m_initFunction = *initAverageCPU;
        m_deleteFunction = *deleteAverageCPU;
        m_updateFunction = *updateAverageCPU;
        break;

    case COMPLETE:
        m_dataObjectRequired = false;
        m_updateFunction = *updateComplete;
        break;
    }
}


EfficientAHC::~EfficientAHC()
{
    freeMem();
    for (unsigned int dIndex = 0; dIndex < m_dendroNodes.size(); ++dIndex) {
        delete m_dendroNodes[dIndex];
    }
}


bool EfficientAHC::freeMem()
{
    ROS_DEBUG_NAMED("EfficientAHC", "EfficientAHC::%s", __func__);
    delete[] I;

    if (m_dataObjectRequired) {
        for (unsigned int row = 0; row < m_allocatedN; ++row) {
            for (unsigned int col = 0; col < m_allocatedN; ++col) {
                m_deleteFunction(CC[row][col].m_data);
                CC[row][col].m_data = NULL;
            }
            delete[] CC[row];
            fh_deleteheap(P[row]);
        }
    }
    else {
        for (unsigned int row = 0; row < m_allocatedN; ++row) {
            delete[] CC[row];
            fh_deleteheap(P[row]);
        }
    }

    delete[] CC;
    delete[] P;
    delete[] m_rootNodes;

    ROS_DEBUG_NAMED("EfficientAHC", "EfficientAHC::%s done!", __func__);
    return true;
}


bool EfficientAHC::resize(unsigned int size)
{
    ROS_DEBUG_NAMED("EfficientAHC", "EfficientAHC::%s( %i )", __func__, size);
    freeMem();

    CC = new Cost*[size];
    I = new bool[size];
    P = new fibheap*[size];

    if (m_dataObjectRequired) {
        for (unsigned int row = 0; row < size; ++row) {
            CC[row] = new Cost[size];
            for (unsigned int col = 0; col < size; ++col) {
                m_createFunction(CC[row][col].m_data);
            }
            P[row] = fh_makekeyheap();
        }
    }
    else {
        for (unsigned int row = 0; row < size; ++row) {
            CC[row] = new Cost[size];
            P[row] = fh_makekeyheap();
        }
    }

    m_rootNodes = new DendroNode*[size];
    m_allocatedN = size;

    ROS_DEBUG_NAMED("EfficientAHC", "EfficientAHC::%s done!", __func__);
    return true;
}


void EfficientAHC::initializeSingle(const std::vector<Point2D>& points)
{
    std::vector<unsigned int> pointIndices;
    pointIndices.push_back(0);

    for (unsigned int pIndex = 1; pIndex < points.size(); ++pIndex) {
        double distance = (points[pointIndices.back()] - points[pIndex]).norm();

        if (distance >= m_threshold) {
            m_dendroNodes.push_back(new DendroNode(pointIndices));
            pointIndices.clear();
        }
        pointIndices.push_back(pIndex);
    }

    m_dendroNodes.push_back(new DendroNode(pointIndices));
    N = m_dendroNodes.size();
    if (m_allocatedN < N) {
        resize(N);
    }

    for (unsigned int row = 0; row < N; ++row) {
        m_rootNodes[row] = m_dendroNodes[row];
        fh_clearheap(P[row]);
        I[row] = true;

        // upper part of the matrix
        for (unsigned int col = row + 1; col < N; ++col) {
            double distance = getMinDistance(m_dendroNodes[row], m_dendroNodes[col], points);

            // set distance and index
            CC[row][col].m_distance = distance;
            CC[row][col].m_index = col;
            fibheap_el* node = fh_insertkey(P[row], CC[row][col].m_distance * DOUBLE_TO_INT_MULTIPLIER, &CC[row][col]);
            CC[row][col].m_node = node;
        }

        // lower part of the matrix, copy distances from upper part
        for (unsigned int col = 0; col < row; ++col) {
            double distance = CC[col][row].m_distance;

            // set distance and index
            CC[row][col].m_distance = distance;
            CC[row][col].m_index = col;
            fibheap_el* node = fh_insertkey(P[row], CC[row][col].m_distance * DOUBLE_TO_INT_MULTIPLIER, &CC[row][col]);
            CC[row][col].m_node = node;
        }
    }
}


void EfficientAHC::initializeAverage(const std::vector<Point2D>& points)
{
    N = points.size();

    if (m_allocatedN < N) {
        resize(N);
    }

    for (unsigned int row = 0; row < N; ++row) {
        fh_clearheap(P[row]);
        I[row] = true;
        m_rootNodes[row] = new DendroNode(row);
        m_dendroNodes.push_back(m_rootNodes[row]);

        // upper part of the matrix
        for (unsigned int col = row + 1; col < N; ++col) {
            double distance = 0.0;
            CC[row][col].m_index = col;
            DataAverageCPU* da = new DataAverageCPU;
            CC[row][col].m_data = da;
            da->m_rowSize = 1;
            da->m_colSize = 1;

            distance = (points[row] - points[col]).norm();
            da->m_rowSum = points[row];
            da->m_colSum = points[col];

            CC[row][col].m_distance = distance;
            fibheap_el* node = fh_insertkey(P[row], CC[row][col].m_distance * DOUBLE_TO_INT_MULTIPLIER, &CC[row][col]);
            CC[row][col].m_node = node;
        }

        // lower part of the matrix, copy distances from upper part
        for (unsigned int col = 0; col < row; ++col) {
            const Cost& upper = CC[col][row];
            double distance = upper.m_distance;
            CC[row][col].m_distance = distance;
            CC[row][col].m_index = col;

            DataAverageCPU* da = new DataAverageCPU;
            CC[row][col].m_data = da;
            da->m_rowSize = 1;
            da->m_rowSum = ((DataAverageCPU*) upper.m_data)->m_colSum;
            da->m_colSize = 1;
            da->m_colSum = ((DataAverageCPU*) upper.m_data)->m_rowSum;

            fibheap_el* node = fh_insertkey(P[row], CC[row][col].m_distance * DOUBLE_TO_INT_MULTIPLIER, &CC[row][col]);
            CC[row][col].m_node = node;
        }
    }
}


void EfficientAHC::initializeComplete(const std::vector<Point2D>& points)
{
    N = points.size();

    if (m_allocatedN < N) {
        resize(N);
    }

    for (unsigned int row = 0; row < N; ++row) {
        fh_clearheap(P[row]);
        I[row] = true;

        m_rootNodes[row] = new DendroNode(row);
        m_dendroNodes.push_back(m_rootNodes[row]);

        // upper part of the matrix
        for (unsigned int col = row + 1; col < N; ++col) {
            double distance = (points[row] - points[col]).norm();

            CC[row][col].m_distance = distance;
            CC[row][col].m_index = col;
            fibheap_el* node = fh_insertkey(P[row], CC[row][col].m_distance * DOUBLE_TO_INT_MULTIPLIER, &CC[row][col]);
            CC[row][col].m_node = node;
        }

        // lower part of the matrix, copy distances from upper part
        for (unsigned int col = 0; col < row; ++col) {
            double distance = CC[col][row].m_distance;

            CC[row][col].m_distance = distance;
            CC[row][col].m_index = col;
            fibheap_el* node = fh_insertkey(P[row], CC[row][col].m_distance * DOUBLE_TO_INT_MULTIPLIER, &CC[row][col]);
            CC[row][col].m_node = node;
        }
    }
}


void EfficientAHC::buildDendrogram(const Eigen::MatrixXd& data)
{
    ROS_DEBUG_NAMED("EfficientAHC", "EfficientAHC::%s", __func__);

    // for the levels in the dendrogram
    bool quit = false;
    if (N < 2) {
        return;
    }

    for (unsigned int k = 0; k < N - 1 && !quit; ++k) {
        ROS_DEBUG_NAMED("EfficientAHC", "   level: %i", k);

        // find minimum distance
        unsigned int k1 = UINT_MAX; // also used as row index
        double minDistance = DBL_MAX;
        for (unsigned int row = 0; row < N; ++row) {
            if (I[row] /*&& I[((Cost*) fh_min(P[row]))->m_index]*/
            && ((Cost*) fh_min(P[row]))->m_distance < minDistance) {
                if (!I[((Cost*) fh_min(P[row]))->m_index]) {
                    ROS_FATAL("Wrong element in P[%i]!", row);
                    std::terminate();
                    fh_extractmin(P[row]);
                }
                else {
                    minDistance = ((Cost*) fh_min(P[row]))->m_distance;
                    k1 = row;
                    ROS_DEBUG_NAMED("EfficientAHC", "      new k1: %i, %f, k2: %i in %p", k1, minDistance, ((Cost* ) fh_min(P[row]))->m_index, P[row]);
                }
            }
        }

        if (minDistance > m_threshold) {
            quit = true;
        }

        unsigned int k2 = ((Cost*) fh_min(P[k1]))->m_index;
        assert(k1 != k2);
        assert(I[k1]);
        assert(I[k2]);

        ROS_DEBUG_NAMED("EfficientAHC", "      min: %f (%i, %i)", minDistance, k1, k2);
        I[k2] = false; // disable priority queue in row k2 and column k2

        // join DendroNodes k1 and k2 into k1
        m_rootNodes[k1] = new DendroNode(m_rootNodes[k1], m_rootNodes[k2], minDistance);
        m_dendroNodes.push_back(m_rootNodes[k1]);

        // clear queue P[k1]
        fh_deleteheap(P[k1]);
        P[k1] = fh_makekeyheap();
        assert(fh_min(P[k1]) == NULL);

        // update distance matrix and queues
        ROS_DEBUG_NAMED("EfficientAHC", "      update distances:");
        for (unsigned int row = 0; row < N; ++row) {
            if (I[row] && row != k1)
            {
                // delete
                {
                    ROS_DEBUG_NAMED("EfficientAHC", "         1 delete node: <%i, %i> from %p", row, CC[row][k1].m_index, P[row]);
                    fh_replacekey(P[row], CC[row][k1].m_node, DELETE_KEY);
                    fh_extractmin(P[row]);
                }

                // delete
                {
                    ROS_DEBUG_NAMED("EfficientAHC", "         2 delete node: <%i, %i> from %p", row, CC[row][k2].m_index, P[row]);
                    fh_replacekey(P[row], CC[row][k2].m_node, DELETE_KEY);
                    fh_extractmin(P[row]);
                }

                // update distances
                m_updateFunction(data, CC[row][k1], CC[row][k2], CC[row][k1], CC[k1][row]);

                // insert
                {
                    ROS_DEBUG_NAMED("EfficientAHC", "         1 insert node: <%i, %i>", row, CC[row][k1].m_index);
                    fibheap_el* node = fh_insertkey(P[row], CC[row][k1].m_distance * DOUBLE_TO_INT_MULTIPLIER, &CC[row][k1]);
                    CC[row][k1].m_node = node;
                }

                // insert
                {
                    ROS_DEBUG_NAMED("EfficientAHC", "         2 insert node: <%i, %i>", k1, CC[k1][row].m_index);
                    fibheap_el* node = fh_insertkey(P[k1], CC[k1][row].m_distance * DOUBLE_TO_INT_MULTIPLIER, &CC[k1][row]);
                    CC[k1][row].m_node = node;
                }
            }
        }
    }
}


double EfficientAHC::getMinDistance(DendroNode* left, DendroNode* right, const std::vector<Point2D>& points) const
{
    const std::vector<unsigned int>& ptsLeft = left->getPoints();
    const std::vector<unsigned int>& ptsRight = right->getPoints();

    double minDistance = DBL_MAX;
    for (unsigned int lIndex = 0; lIndex < ptsLeft.size(); ++lIndex) {
        for (unsigned int rIndex = 0; rIndex < ptsRight.size(); ++rIndex) {
            double xdiff = points[ptsLeft[lIndex]](0) - points[ptsRight[rIndex]](0);
            double ydiff = points[ptsLeft[lIndex]](1) - points[ptsRight[rIndex]](1);
            double distance = hypot(xdiff, ydiff);
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
    }
    return minDistance;
}


//--- Function type: create a data object. ---//

void EfficientAHC::createAverageCPU(void*& object)
{
    assert(object == NULL);
    object = new DataAverageCPU;
}

void EfficientAHC::createAverageMEM(void*& object)
{
    assert(object == NULL);
    object = new DataAverageMEM;
}


//--- Function type: initializes a data object. ---//

void EfficientAHC::initAverageCPU(const Eigen::MatrixXd& data, unsigned int row, unsigned int col, void*& object)
{
    DataAverageCPU* da = (DataAverageCPU*) object;
    da->m_rowSize = 1;
    da->m_colSize = 1;
    da->m_rowSum = data.col(row);
    da->m_colSum = data.col(col);
}

void EfficientAHC::initAverageMEM(const Eigen::MatrixXd& data, unsigned int row, unsigned int col, void*& object)
{
    DataAverageMEM* da = (DataAverageMEM*) object;
    da->m_rowIndices.clear();
    da->m_rowIndices.push_back(row);
    da->m_colIndices.clear();
    da->m_colIndices.push_back(col);
}


//--- Function type: delete a data object. ---//

void EfficientAHC::deleteAverageCPU(void* object)
{
    delete (DataAverageCPU*) object;
}

void EfficientAHC::deleteAverageMEM(void* object)
{
    delete (DataAverageMEM*) object;
}


//--- Function type: update distances. ---//

void EfficientAHC::updateSingle(const Eigen::MatrixXd& unused, const Cost& row_k1_in, const Cost& row_k2_in, Cost& row_k1_out, Cost& k1_row_out)
{
    double tmp = std::min(row_k1_in.m_distance, row_k2_in.m_distance);
    row_k1_out.m_distance = tmp;
    k1_row_out.m_distance = tmp;
}

void EfficientAHC::updateAverageCPU(const Eigen::MatrixXd& unused, const Cost& row_k1_in, const Cost& row_k2_in, Cost& row_k1_out, Cost& k1_row_out)
{
    DataAverageCPU* row_k1_in_da = (DataAverageCPU*) row_k1_in.m_data;
    DataAverageCPU* row_k2_in_da = (DataAverageCPU*) row_k2_in.m_data;
    DataAverageCPU* row_k1_out_da = (DataAverageCPU*) row_k1_out.m_data;
    DataAverageCPU* k1_row_out_da = (DataAverageCPU*) k1_row_out.m_data;

    // update vector sums and sizes
    row_k1_out_da->m_rowSum = row_k1_in_da->m_rowSum;
    row_k1_out_da->m_rowSize = row_k1_in_da->m_rowSize;
    row_k1_out_da->m_colSum = row_k1_in_da->m_colSum + row_k2_in_da->m_colSum;
    row_k1_out_da->m_colSize = row_k1_in_da->m_colSize + row_k2_in_da->m_colSize;

    k1_row_out_da->m_rowSum = row_k1_out_da->m_colSum;
    k1_row_out_da->m_rowSize = row_k1_out_da->m_colSize;
    k1_row_out_da->m_colSum = row_k1_out_da->m_rowSum;
    k1_row_out_da->m_colSize = row_k1_out_da->m_rowSize;

    // calculate distance
    Eigen::VectorXd tmp = row_k1_out_da->m_rowSum / row_k1_out_da->m_rowSize - row_k1_out_da->m_colSum / row_k1_out_da->m_colSize;
    row_k1_out.m_distance = tmp.norm();
    k1_row_out.m_distance = row_k1_out.m_distance;
}

void EfficientAHC::updateComplete(const Eigen::MatrixXd& unused, const Cost& row_k1_in, const Cost& row_k2_in, Cost& row_k1_out, Cost& k1_row_out)
{
    double tmp = std::max(row_k1_in.m_distance, row_k2_in.m_distance);
    row_k1_out.m_distance = tmp;
    k1_row_out.m_distance = tmp;
}


} // end of namespace srl_laser_segmentation
