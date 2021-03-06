#include "BaxterController.h"

using namespace std;
using namespace pcl;

BaxterController::BaxterController() : 
	leftArmGroup("left_arm"),
	rightArmGroup("right_arm")
{
	updateStates();

	//Construct joints vector
	vector<string> jointsNames = leftArmGroup.getActiveJoints();
	BOOST_FOREACH(string joint, jointsNames)
	{
		leftJoints.push_back(JointInfo(joint, 0.0f));
	}

	jointsNames.clear();
	jointsNames = rightArmGroup.getActiveJoints();
	BOOST_FOREACH(string joint, jointsNames)
	{
		rightJoints.push_back(JointInfo(joint, 0.0f));
	}

	setArmOriginalPosition(LEFT_ARM);
	setArmOriginalPosition(RIGHT_ARM);
}

BaxterController::~BaxterController()
{
	// do nothing
}

bool BaxterController::setArmOriginalPosition(Arm arm)
{
	vector<double> joints;
	for (int i = 0; i < 7; i++)
		joints.push_back(0.0);

	setJoints(arm, joints);
}

bool BaxterController::moveJoint(Arm arm, string joint, double value)
{
	updateStates();
	if (arm)
	{
		cout << "First value" << value << endl;
		value += curLeftArmState->getJointPositions("left_" +joint)[0];
		cout << "Second value" << value << " left_" +joint << endl;
		curLeftArmState->setJointPositions("left_" + joint, &value);
		leftArmGroup.setJointValueTarget(*curLeftArmState);
		leftArmGroup.move();
	} else
	{
		value += curLeftArmState->getJointPositions("right_" +joint)[0];
		curRightArmState->setJointPositions("right_" +joint, &value);
		leftArmGroup.setJointValueTarget(*curRightArmState);
		rightArmGroup.move();
	}
#ifdef DEBUG
	ros::Duration(3.0).sleep();
#endif
	return true;
}

bool BaxterController::moveArm(Arm arm, double x, double y, double z)
{
	geometry_msgs::PoseStamped poseStamped;
	if (arm)
	{
		poseStamped = leftArmGroup.getCurrentPose("left_gripper");
		cout << poseStamped.pose << endl;
		poseStamped.pose.position.x += x;
		poseStamped.pose.position.y += y;
		poseStamped.pose.position.z += z;
		cout << poseStamped.pose << endl;

		poseStamped.header.seq++;
		poseStamped.header.stamp = ros::Time::now();

		leftArmGroup.setJointValueTarget(poseStamped.pose, "left_gripper");
		leftArmGroup.move();

	} else
	{
		poseStamped = rightArmGroup.getCurrentPose("right_gripper");
		cout << poseStamped.pose << endl;
		poseStamped.pose.position.x += x;
		poseStamped.pose.position.y += y;
		poseStamped.pose.position.z += z;
		cout << poseStamped.pose << endl;

		poseStamped.header.seq++;
		poseStamped.header.stamp = ros::Time::now();

		rightArmGroup.setJointValueTarget(poseStamped.pose, "right_gripper");
		rightArmGroup.move();
	}
#ifdef DEBUG
	ros::Duration(0.5).sleep();
#endif
	return true;
}

bool BaxterController::setArm(Arm arm, Eigen::Vector4f & point)
{
	if (arm) //Left arm
	{
		leftArmGroup.setPositionTarget(point(0), point(1), point(2), "left_gripper");
		leftArmGroup.move();
	} else 
	{
		rightArmGroup.setPositionTarget(point(0), point(1), point(2), "right_gripper");
		rightArmGroup.move();
	}
#ifdef DEBUG
	ros::Duration(2.0).sleep();
#endif
	return true;
}

bool BaxterController::setArm(Arm arm, double x, double y, double z)
{
	if (arm) //Left arm
	{
		leftArmGroup.setPositionTarget(x, y, z, "left_gripper");
		leftArmGroup.move();
	} else
	{
		rightArmGroup.setPositionTarget(x, y, z, "right_gripper");
		rightArmGroup.move();
	}
#ifdef DEBUG
	ros::Duration(2.0).sleep();
#endif
	return true;
}


bool BaxterController::setArmPose(Arm arm, double x, double y, double z)
{
	if (arm) //Left arm
	{
		geometry_msgs::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;

		pose.orientation.x = -0.0133155;
		pose.orientation.y = 0.909243;
		pose.orientation.z = -0.343736;
		pose.orientation.w = 0.234406;

		leftArmGroup.setPoseTarget(pose, "left_gripper");
		leftArmGroup.move();
	} else
	{
		geometry_msgs::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;

		pose.orientation.x = -0.0133155;
		pose.orientation.y = -0.909243;
		pose.orientation.z = -0.343736;
		pose.orientation.w = -0.234406;

		rightArmGroup.setPoseTarget(pose, "right_gripper");
		rightArmGroup.move();

	}
#ifdef DEBUG
	ros::Duration(2.0).sleep();
#endif
	return true;
}


bool BaxterController::setJoint(Arm arm, string joint, double value)
{
	if (arm)
	{
		curLeftArmState->setJointPositions("left_" +joint, &value);
	} else
	{
		curRightArmState->setJointPositions("right_" +joint, &value);
	}
#ifdef DEBUG
	ros::Duration(3.0).sleep();
#endif
	return true;
}

bool BaxterController::setJoints(Arm arm, vector<double> & jointsValues)
{
	// Update values of each joint
	int i = 0;

	if (arm) // Left arm
	{
		BOOST_FOREACH(JointInfo & jInfo, leftJoints)
		{
			jInfo.jointValue = jointsValues[i++];
			curLeftArmState->setJointPositions(jInfo.jointName, &(jInfo.jointValue)) ;
		}

		leftArmGroup.setJointValueTarget(*curLeftArmState);
		leftArmGroup.move();
	} else
	{
		BOOST_FOREACH(JointInfo & jInfo, rightJoints)
		{
			jInfo.jointValue = jointsValues[i++];
			curRightArmState->setJointPositions(jInfo.jointName, &(jInfo.jointValue)) ;
		}

		rightArmGroup.setJointValueTarget(*curRightArmState);
		rightArmGroup.move();
	}
#ifdef DEBUG
	ros::Duration(2.0).sleep();
#endif
	return true;
}

void BaxterController::printArmsState()
{
	/* Update states */
	updateStates();

	/* Left arm */
	cout << "Left arm state: " << endl;

	vector<string> joints = leftArmGroup.getActiveJoints();
	BOOST_FOREACH(std::string joint, joints)
	{
		std::cout << joint << ":\t" << curLeftArmState->getJointPositions(joint)[0] << std::endl;
	}

	/* Right arm */
	cout << "Right arm state: " << endl;

	joints.clear();
	joints = rightArmGroup.getActiveJoints();
	BOOST_FOREACH(std::string joint, joints)
	{
		std::cout << joint << ":\t" << curRightArmState->getJointPositions(joint)[0] << std::endl;
	}
}

void BaxterController::updateStates()
{
	curLeftArmState = leftArmGroup.getCurrentState();
	curRightArmState = rightArmGroup.getCurrentState();
}