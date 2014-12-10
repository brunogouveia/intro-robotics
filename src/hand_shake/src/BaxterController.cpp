#include "BaxterController.h"

using namespace std;
using namespace pcl;

BaxterController::BaxterController() : 
	leftArmGroup("left_arm"),
	rightArmGroup("right_arm")
{
	curState = leftArmGroup.getCurrentState();

	//Construct joints vector
	vector<string> jointsNames = leftArmGroup.getActiveJoints();
	BOOST_FOREACH(string joint, jointsNames)
	{
		joints.push_back(JointInfo(joint, 0.0f));
	}
}

BaxterController::~BaxterController()
{
	// do nothing
}

bool BaxterController::setArm(Arm arm, Eigen::Vector4f & point)
{
	//do nothing yet
	leftArmGroup.setPositionTarget(point(0), point(1), point(2), "left_gripper");
	leftArmGroup.move();
}

bool BaxterController::setArm(Arm arm, vector<double> & jointsValues)
{
	// Update values of each joint
	int i = 0;
	BOOST_FOREACH(JointInfo & jInfo, joints)
	{
		jInfo.jointValue = jointsValues[i++];
		curState->setJointPositions(jInfo.jointName, &(jInfo.jointValue)) ;
	}

	leftArmGroup.setJointValueTarget(*curState);
	leftArmGroup.move();
}