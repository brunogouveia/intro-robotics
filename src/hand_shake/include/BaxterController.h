#ifndef BAXTER_CONTROLLER__
#define BAXTER_CONTROLLER__  value

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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionIK.h>
#include <baxter_core_msgs/SolvePositionIK.h>

#include <iostream>
#include <sstream>

#include <body_msgs/Hand.h>
#include <Eigen/Geometry>

#define DEBUG 1

#define RIGHT_ARM	0
#define LEFT_ARM	1
typedef int Arm;

using namespace pcl;
using namespace std;

class JointInfo
{
public:
	string jointName;
	double jointValue;

	JointInfo(string name, double value): jointName(name), jointValue(value) {};
	~JointInfo(){}
	
};

class BaxterController
{
private:
	moveit::planning_interface::MoveGroup leftArmGroup;
	moveit::planning_interface::MoveGroup rightArmGroup;
	robot_state::RobotStatePtr curLeftArmState;
	robot_state::RobotStatePtr curRightArmState;

	vector<JointInfo> leftJoints;
	vector<JointInfo> rightJoints;

public:
	BaxterController();
	~BaxterController();

	bool setArmOriginalPosition(Arm arm);

	bool moveJoint(Arm arm, string joint, double value);
	bool moveArm(Arm arm, double x, double y, double z);

	bool setArm(Arm arm, Eigen::Vector4f & point);
	bool setArm(Arm arm, double x, double y, double z);
	bool setArmPose(Arm arm, double x, double y, double z);
	
	bool setJoint(Arm arm, string joint, double value);
	bool setJoints(Arm arm, vector<double> & joints);

	void printArmsState();
	void updateStates();

};

#endif