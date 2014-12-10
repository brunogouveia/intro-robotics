#ifndef HAND_DETECTOR__
#define HAND_DETECTOR__ value

#include "ros/ros.h"
#include "boost/foreach.hpp"
#include <math.h>
#include <string.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <pcl_tools/pcl_utils.h>
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
#include "nnn/nnn.hpp"

#include <iostream>
#include <sstream>

#include <body_msgs/Hand.h>

#define LIST_SIZE 		20		//Number of points used to compute mean and variance
#define MAX_VARIANCE 	0.0015f	//If variance is greate than that, its too noisy

using namespace pcl;
using namespace std;

class Params 
{
public:
	uint listSize;
	float maxVariance;

	Params() : listSize(LIST_SIZE), maxVariance(MAX_VARIANCE) {}
};

class HandDetector {
private: 
	ros::Publisher pointCloudPub;
	ros::Publisher markerPub;
	ros::Publisher handPub;

	ros::Subscriber pointCloudSub;

	body_msgs::Hand hand;

public:
	Params params;
	
	HandDetector() {}

	~HandDetector() {}

	void init();

	bool isHand(Eigen::Vector4f & handCenter);

	void getSubCloud(PointCloud<PointXYZ> & cloudin, vector<int> & inds, PointCloud<PointXYZ> & cloudout);

	void pubMarker(Eigen::Vector4f & point);

	void pclCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr);
};

#endif