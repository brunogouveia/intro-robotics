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

#include "HandDetector.h"

using namespace pcl;
using namespace std;



// void init(int argc, char ** argv)
// {
// 	if (argc == 1) return;

// 	int i = 1;
// 	while (argc > i)
// 	{
// 		if (!strcmp(argv[i], "-mv"))
// 		{
// 			if ((argc - i) == 1) { cout << "Missing value of maxVariance" << endl; exit(1);}
// 			params.maxVariance = atof(argv[++i]);
// 		}
// 		else if (!strcmp(argv[i], "-ll"))
// 		{
// 			if ((argc - i) == 1) { cout << "Missing value of listSize" << endl; exit(1);}
// 			params.listSize = atoi(argv[++i]);
// 		}
// 	}
// }

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.	The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "HandDetection");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	// ros::NodeHandle n;

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.	advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().	Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.	If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	// camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
	// image_pub = n.advertise<sensor_msgs::Image>("image_rect", 1);

	// ros::Subscriber camera_info_sub = n.subscribe("camera/depth/camera_info", 1, cameraCallback)
	// ros::Subscriber image_sub = n.subscribe("camera/depth/image", 1, imageCallback);

	 HandDetector hd;
	 hd.init();

	ros::spin();

	return 0;
}