#include <BaxterController.h>
#include <iostream>
#include <fstream>
#include <string>

BaxterController * bcPtr;


void callBack(const body_msgs::Hand::ConstPtr hand);
bool testFile(const char * fileName);

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
	ros::init(argc, argv, "BaxterControllerNode");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	BaxterController bc;
	bcPtr = &bc;
	ros::Subscriber sub;

	/*
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
	 
	 bc.printArmsState();
	 // ros::sleep(3);

	if (argc >= 2)
	{
		testFile(argv[1]);
	} else 
	{
		// sub = n.subscribe("/hand1", 1, callBack);
	}

	body_msgs::Hand hand;

	hand.id = 0;
	hand.position.x = 0.895556;
	hand.position.y = 0.292756;
	hand.position.z = -0.00392263;
	callBack(boost::shared_ptr<body_msgs::Hand const>(&hand));



	// vector<double> joints;
	// joints.push_back(-0.459);
	// joints.push_back(-0.102);
	// joints.push_back(1.8);
	// joints.push_back(1.7);
	// joints.push_back(0.9);
	// joints.push_back(0);
	// joints.push_back(0.2);

	// bc.setJoints(LEFT_ARM, joints);
	// cout<< "Terminou" << endl;


	ros::waitForShutdown();
}

void callBack(const body_msgs::Hand::ConstPtr hand)
{
	static int lastHandId = 0;
	if (hand->id != lastHandId || lastHandId == 0)
	{
		ROS_INFO("Hand received - starting");

		bool r = bcPtr->setArm(LEFT_ARM, hand->position.x,hand->position.y,hand->position.z);

		ROS_INFO("Hand received - First point");
		bcPtr->moveArm(LEFT_ARM, 0, 0, 0.1);

		ROS_INFO("Hand received - Second point");
		bcPtr->moveArm(LEFT_ARM, 0, 0, -0.2);

		ROS_INFO("Hand received - Third point");
		bcPtr->moveArm(LEFT_ARM, 0, 0, 0.1);

		lastHandId = hand->id;
		ROS_INFO("Going back to original position");
	}
}

bool testFile(const char * fileName)
{
	string line;
	ifstream inputFile(fileName);

	if (inputFile.is_open())
	{
		while (getline(inputFile, line))
		{
			float x,y,z;
			sscanf(line.c_str(), "%f %f %f", &x, &y, &z);
			cout << line.c_str() << endl;
			cout << "Going to point: " << x << ", " << y << ", " << z << endl;
			Eigen::Vector4f point(x, y, z,1.0f);
			bcPtr->setArm(LEFT_ARM, point);
		}
	}
	else
		cout << "Unable to open file." << endl;

}