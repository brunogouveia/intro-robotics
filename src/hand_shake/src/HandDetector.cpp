#include "HandDetector.h"

using namespace pcl;
using namespace std;



void HandDetector::init()
{
	ros::NodeHandle nh;

	pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/camera/rgb/points", 1);
	markerPub	  = nh.advertise<visualization_msgs::Marker>("/marker", 1);
	handPub		  = nh.advertise<body_msgs::Hand>("/hand1", 1);
	pointCloudSub = nh.subscribe("/camera/depth/points", 1, &HandDetector::pclCallback, this);
}


bool HandDetector::isHand(Eigen::Vector4f & handCenter) 
{
	static list<Eigen::Vector4f> points;
	static Eigen::Vector4f sum(0.0f,0.0f,0.0f,1.0f);

	//Add the handCenter
	sum += handCenter;
	points.push_front(handCenter);

	//Queue must be full first
	if (points.size() < params.listSize)
		return false;

	//Delete the oldest point
	sum -= points.back();
	points.pop_back();


	//Calculate mean and std
	Eigen::Vector4f mean = sum / LIST_SIZE;

	//Calculate var
	Eigen::Vector4f var(0.0f, 0.0f, 0.0f, 1.0f);
	BOOST_FOREACH(Eigen::Vector4f point, points) 
	{
		var += (point - mean).cwiseProduct(point - mean);
	}
	var /= LIST_SIZE;
	Eigen::Vector3f v(var(0), var(1), var(2));

	cout << v.norm() << endl;
	return (v.norm() < params.maxVariance);
}

void HandDetector::getSubCloud(PointCloud<PointXYZ> & cloudin, vector<int> & inds, PointCloud<PointXYZ> & cloudout) {
	pcl::ExtractIndices<PointXYZ> extract;
	extract.setInputCloud (cloudin.makeShared());
	extract.setIndices (boost::make_shared<vector<int> > (inds));
	extract.setNegative (false);
	extract.filter (cloudout);
}

void HandDetector::pubMarker(Eigen::Vector4f & point) {
	uint32_t shape = visualization_msgs::Marker::CUBE;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/camera_depth_frame";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = point(2);
	marker.pose.position.y = -point(0);
	marker.pose.position.z = -point(1);
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = .05;
	marker.scale.y = .05;
	marker.scale.z = .05;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	cout << point(0) << " " << point(1) << " " << point(2) << endl;

	markerPub.publish(marker);
}

void HandDetector::pclCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr)
{
	static bool sendingHand = false;
	PointCloud<PointXYZ> cloud = *ptr;
	vector<PointCloud<PointXYZ> > initialclouds;
	vector<int> inds3(cloud.size(),1);

	//Find points near to camera
	Eigen::Vector4f camera(0.0f,0.0f,0.0f,1.0f);
	vector<int> inds;
	vector<float> distances;

	// NNN(cloud, camera, inds, distances, 1.0f);
	Eigen::Vector4f closestPoint = Knn::pointsInBallWithClosest(cloud,camera, 1.0f, inds);
	
	// int index=0; double smallestdist;
	// for(uint i=0; i < distances.size(); ++i){
	// 	if(distances[i] < smallestdist || i == 0 ){
	// 		index = inds[i];
	// 		smallestdist = distances[i];
	// 	}
	// }
	// smallestdist = sqrt(smallestdist);
	// PointXYZ closestPoint = cloud.points[index];

	// NNN(cloud, closestPoint, inds, .2);
	Knn::pointsInBall(cloud, closestPoint, 0.2, inds);


	//if there is nothing near that point, we're probably seeing noise.  just give up
	if(inds.size() < 100){
		std::cout<<"very few points " << endl;
		return;
	}

	Eigen::Vector4f centroid;
	// PointXYZ centroidPoint;

	compute3DCentroid(cloud, inds, centroid);
	centroid(1) -= 0.02;
	// centroidPoint.x = centroid(0);
	// centroidPoint.y = centroid(1)-.02;
	// centroidPoint.z = centroid(2);

	// NNN(cloud, centroidPoint, inds, .1);
	Knn::pointsInBall(cloud, centroid, 0.1, inds);

	PointCloud<PointXYZ> cloudout;
	getSubCloud(cloud,inds,cloudout);
	// //-------Decide whether we are looking at potential hands:
	// //try to classify whether this is actually a hand, or just a random object (like a face)
	// //if there are many points at the same distance that we did not grab, then the object is not "out in front"
	// for(uint i=0;i<inds.size(); ++i) inds3[inds[i]]=0; //mark in inds3 all the points in the potential hand
	// pcl::compute3DCentroid(cloudout,centroid);
	// int s1,s2=0;
	// s1=inds.size();
	// //search for all points in the cloud that are as close as the center of the potential hand:
	// NNN(cloud,camera,inds, centroid.norm());
	// for(uint i=0;i<inds.size(); ++i){
	// 	if(inds3[inds[i]]) ++s2;
	// }
	// if(((float)s2)/((float)s1) > 25){
	// 	std::cout<<"No hands detected " << ((float)s2)/((float)s1)<< endl;
	// 	//return false; //uncomment
	// 	return;
	// }

	

	if (inds.size() > 1000) {
		if (isHand(centroid)) {
			pubMarker(centroid);
			pointCloudPub.publish(cloudout);

			if (!sendingHand)
			{
				//Publish hand
				geometry_msgs::Point point;
				point.x = centroid(2);
				point.y = -centroid(0);
				point.z = -centroid(1);

				hand.id++;
				hand.position = point;
				hand.stamp = ros::Time::now();
				handPub.publish(hand);
				sendingHand = true;
			}
		} else
		{
			sendingHand = false;
		}
	}
}

