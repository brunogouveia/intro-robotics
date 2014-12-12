#ifndef KNN__
#define KNN__ value

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/extract_indices.h>

#include <boost/foreach.hpp>


using namespace pcl;

class Knn
{
public:
	Knn();
	~Knn();

	/**
	 *	Return the index [inds] of all point in the clound [cloud] that is inside the ball [center, r]
	 */
	static void pointsInBall(PointCloud<PointXYZ> & cloud, Eigen::Vector4f & center, float r,  std::vector<int> & inds);

	static Eigen::Vector4f pointsInBallWithClosest(PointCloud<PointXYZ> & cloud, Eigen::Vector4f & center, float r,  std::vector<int> & inds);	
};

#endif