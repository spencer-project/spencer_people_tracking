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

#ifndef SRL_LASER_SEGMENTATION_EFFICIENT_AHC_H_
#define SRL_LASER_SEGMENTATION_EFFICIENT_AHC_H_

#include <srl_laser_segmentation/efficient_ahc/fib.h>
#include <Eigen/Core>
#include <vector>
#include <limits.h>
#include <float.h>
#include <stdio.h>



namespace srl_laser_segmentation {

typedef Eigen::Vector2d Point2D;


/**
 * Node in the dendrogram.
 *
 * @author Matthias Luber
 */
class DendroNode
{
public:
	/** Constructor.
	 * @param index Index of the corresponding data point. */
	DendroNode(unsigned int index);

	/** Constructor.
	 * @param indices Set of indices of the corresponding data points. */
	DendroNode(const std::vector<unsigned int>& indices);

	/** Constructor to merge two nodes.
	 * @param left 1st node to merge.
	 * @param right 2nd node to merge.
	 * @param distance Distance between these nodes. */
	DendroNode(DendroNode* left, DendroNode* right, double distance);

	/// Get the left child node.
	inline DendroNode* getLeft() const
	{
		return m_left;
	}

	/// Get the right child node.
	inline DendroNode* getRight() const
	{
		return m_right;
	}

	/// Get the distance between the merged nodes.
	inline double getDistance() const
	{
		return m_distance;
	}

	/// Get the vector of data point indices in this node.
	inline const std::vector<unsigned int>& getPoints() const
	{
		return m_points;
	}


protected:
	/// Left child node in the dendrogram.
	DendroNode* m_left;

	/// Right child node in the dendrogram.
	DendroNode* m_right;

	/// Distance between the left and the right node.
	double m_distance;

	/// Vector of indices of the data points in this node.
	std::vector<unsigned int> m_points;

};


const std::string LinkageString[] = { "SINGLE", "AVERAGE_CPU", "AVERAGE_MEM", "COMPLETE" };


/**
 * EfficienAHC.
 * Complexity: O(N^2 logN)
 *
 * @author Matthias Luber
 */
class EfficientAHC
{
public:
	/// Convert double to int values.
	static const int DOUBLE_TO_INT_MULTIPLIER;

	/// Linkage type.
	enum Linkage
	{
		SINGLE, /* */
		AVERAGE_CPU, /* Fast implementation, but needs more memory. */
		AVERAGE_MEM, /* Memory efficient, but a bit slower. */
		COMPLETE
	};

	/// Constructor.
	EfficientAHC(Linkage linkge, double threshold);

	/// Destructor.
	virtual ~EfficientAHC();

	/// Set clustering threshold.
	inline void setThreshold(double threshold)
	{
		m_threshold = threshold;
	}

	/// Get clustering threshold.
	inline const double getThreshold() const
	{
		return m_threshold;
	}

    /// Initialize similarity matrix and priority queues for single linkage.
    void initializeSingle(const std::vector<Point2D>& points);

    /// Initialize similarity matrix and priority queues for average linkage.
    void initializeAverage(const std::vector<Point2D>& points);

    /// Initialize similarity matrix and priority queues for complete linkage.
    void initializeComplete(const std::vector<Point2D>& points);


protected:
	/**
	 * Entry in the cost matrix.
	 */
	struct Cost
	{
		/// Constructor.
		Cost() :
			m_node(NULL), m_data(NULL)
		{
		}

		/// Distance between element in row and column.
		double m_distance;

		/// Index of the assigned column.
		unsigned int m_index;

		/// Fib heap node.
		fibheap_el* m_node;

		/// Additional data.
		void* m_data;

		/// Operator.
		void operator=(int value)
		{
			m_index = value;
		}

		/// Operator.
		void operator=(double value)
		{
			m_distance = value;
		}
	};

	/**
	 * Additional data used in speed optimized average linkage.
	 */
	struct DataAverageCPU
	{
		/// Sum of the elements in the cluster of the assigned row.
		Eigen::VectorXd m_rowSum;

		/// Number of data points in the cluster of the assigned row.
		unsigned int m_rowSize;

		/// Sum of the elements in the cluster of the assigned column.
		Eigen::VectorXd m_colSum;

		/// Number of data points in the cluster of the assigned column.
		unsigned int m_colSize;
	};

	/**
	 * Additional data used in memory optimized average linkage.
	 */
	struct DataAverageMEM
	{
		/// Indices of the elements in the cluster of the assigned row.
		std::vector<unsigned int> m_rowIndices;

		/// Indices of the elements in the cluster of the assigned column.
		std::vector<unsigned int> m_colIndices;
	};

	/** Resize data structures.
	 * @param size Number of data points to segment. */
	bool resize(unsigned int size);

	/// Free current allocated memory.
	bool freeMem();

	/// Get the minimal distance between the clusters represented by left and right.
	double getMinDistance(DendroNode* left, DendroNode* right, const std::vector<Point2D>& points) const;

	/// Build up the dendrogram up to the EffAHC_CLUSTER_DISTANCE threshold.
	void buildDendrogram(const Eigen::MatrixXd& data);

	/// Linkage type.
	Linkage m_linkage;

	/// Clustering Threshold.
	double m_threshold;

	/// Number of allocated data structures.
	unsigned int m_allocatedN;

	/// Number of data points.
	unsigned int N;

	/// Cost matrix.
	Cost** CC;

	/// Flag, the entries in the cost matrix require an additional data object.
	bool m_dataObjectRequired;

	/// Indicators for fused rows and cols.
	bool* I;

	/// Priority queues with sorted similarities in decreasing order.
	fibheap** P;

	/// Root nodes of the Dendrogram.
	DendroNode** m_rootNodes;

	/// Vector of all instantiated DendroNodes.
	std::vector<DendroNode*> m_dendroNodes;

	/// Function type: create a data object.
	typedef void(*createFunction)(void*& object);

	/// Creates data object used for speed optimized average linkage.
	static void createAverageCPU(void*& object);

	/// Creates data object used for memory optimized average linkage.
	static void createAverageMEM(void*& object);

	/// Function type: initializes a data object.
	typedef void(*initFunction)(const Eigen::MatrixXd& data, unsigned int row, unsigned int col, void*& object);

	/// Initialize data object used for speed optimized average linkage.
	static void initAverageCPU(const Eigen::MatrixXd& data, unsigned int row, unsigned int col, void*& object);

	/// Initialize data object used for memory optimized average linkage.
	static void initAverageMEM(const Eigen::MatrixXd& data, unsigned int row, unsigned int col, void*& object);

	/// Function type: delete a data object.
	typedef void(*deleteFunction)(void* object);

	/// Delete data object used for speed optimized average linkage.
	static void deleteAverageCPU(void* object);

	/// Delete data object used for memory optimized average linkage.
	static void deleteAverageMEM(void* object);

	/// Function type: update distances.
	typedef void(*updateFunction)(const Eigen::MatrixXd& data, const Cost&, const Cost&, Cost&, Cost&);

	/// Update single linkage.
	static void updateSingle(const Eigen::MatrixXd& data, const Cost& row_k1_in, const Cost& row_k2_in, Cost& row_k1_out, Cost& k1_row_out);

	/// Update average linkage.
	static void updateAverageCPU(const Eigen::MatrixXd& data, const Cost& row_k1_in, const Cost& row_k2_in, Cost& row_k1_out, Cost& k1_row_out);

	/// Update average linkage.
	static void updateAverageMEM(const Eigen::MatrixXd& data, const Cost& row_k1_in, const Cost& row_k2_in, Cost& row_k1_out, Cost& k1_row_out);

	/// Update complete linkage.
	static void updateComplete(const Eigen::MatrixXd& data, const Cost& row_k1_in, const Cost& row_k2_in, Cost& row_k1_out, Cost& k1_row_out);

	/// Function pointer to create data function.
	createFunction m_createFunction;

	/// Function pointer to initialize data function.
	initFunction m_initFunction;

	/// Function pointer to delete data function.
	deleteFunction m_deleteFunction;

	/// Function pointer to update function.
	updateFunction m_updateFunction;
};

} // end of namespace srl_laser_segmentation


#endif // SRL_LASER_SEGMENTATION_EFFICIENT_AHC_H_
