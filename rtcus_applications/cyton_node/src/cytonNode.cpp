////// Copyright (c) 2008 Energid Technologies. All rights reserved. ////
//
// $RCSfile: cytonHardwareExample.cpp  $
//
// Description: Example program to exercise the hardwareInterface class
//              of the Cyton arm.
//
// Contents:    
//
/////////////////////////////////////////////////////////////////////////

#include <unistd.h>
#define Sleep(ms) usleep(1000*ms)
#include <string.h>

#include <cytonHardwareInterface.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

using namespace cyton;
using namespace ros;
using namespace std;

/*
 EcBoolean
 testJoints
 (
 cyton::hardwareInterface& hw,
 cyton::StateType jointModifier
 )
 {
 // With a typical setup speed of .25 rad/sec, a complete 2pi revolution would be ~25.12sec
 const EcU32 waitTimeInMS = 30000; // 30 second max wait time
 const size_t numJoints = hw.numJoints();
 if(!numJoints)
 {
 std::cerr << "Invalid configuration. No joints available.\n";
 return EcFalse;
 }

 // Make sure we just have a modifier parameter;
 jointModifier &= ~cyton::JointBaseMask;

 std::cout << "Starting hardware joint test with " << numJoints << " joints\n";
 EcRealVector jointAngle(numJoints), minAngles(numJoints), initAngles(numJoints), maxAngles(numJoints);

 // Pull information from configuration
 std::cout << "Reading angles from configuration.\n";
 if(!hw.getJointStates(minAngles, cyton::MinAngle | jointModifier))
 {
 std::cerr << "Unable to get minimum angles from getJointStates.\n";
 return EcFalse;
 }
 if(!hw.getJointStates(maxAngles, cyton::MaxAngle | jointModifier))
 {
 std::cerr << "Unable to get maximum angles from getJointStates.\n";
 return EcFalse;
 }
 if(!hw.getJointStates(initAngles, cyton::InitAngle | jointModifier))
 {
 std::cerr << "Unable to get init angles from getJointStates.\n";
 return EcFalse;
 }

 // Set all joints to their init value.
 std::cout << "Setting all joints to init angles.\n";
 if(!hw.setJointCommands(0.0, initAngles, jointModifier))
 {
 std::cerr << "Problem setting initialize angles.\n";
 return EcFalse;
 }
 hw.waitUntilCommandFinished(waitTimeInMS);
 Sleep(1000); // Pause for a second to visually verify finished cmd.

 // Start from init setting.
 jointAngle = initAngles;

 // Cycle through each joint individually and go to min, then max, min
 // again and then back to init position.
 for(EcU32 ii=0; ii<numJoints; ++ii)
 {
 jointAngle[ii] = minAngles[ii];
 std::cout << "Setting joint " << ii << " to min angle.\n";
 if(!hw.setJointCommands(30.0+120*ii, jointAngle, jointModifier))
 {
 std::cerr << "Problem setting min angle for joint " << ii << ".\n";
 return EcFalse;
 }
 hw.waitUntilCommandFinished(waitTimeInMS); // Let the hardware achieve its position.
 Sleep(1000); // Pause for a second to visually verify finished cmd.
 
 jointAngle[ii] = maxAngles[ii];
 std::cout << "Setting joint " << ii << " to max angle.\n";
 if(!hw.setJointCommands(60.0+120*ii, jointAngle, jointModifier))
 {
 std::cerr << "Problem setting max angle for joint " << ii << ".\n";
 return EcFalse;
 }
 hw.waitUntilCommandFinished(waitTimeInMS); // Let the hardware achieve its position.
 Sleep(1000); // Pause for a second to visually verify finished cmd.

 jointAngle[ii] = minAngles[ii];
 std::cout << "Setting joint " << ii << " to min angle.\n";
 if(!hw.setJointCommands(90.0+120*ii, jointAngle, jointModifier))
 {
 std::cerr << "Problem setting min angle for joint " << ii << ".\n";
 return EcFalse;
 }
 hw.waitUntilCommandFinished(waitTimeInMS); // Let the hardware achieve its position.
 Sleep(1000); // Pause for a second to visually verify finished cmd.

 
 jointAngle[ii] = initAngles[ii];
 std::cout << "Setting joint " << ii << " to init angle.\n";
 if(!hw.setJointCommands(120.0+120*ii, jointAngle, jointModifier))
 {
 std::cerr << "Problem setting init angle for joint " << ii << ".\n";
 return EcFalse;
 }
 hw.waitUntilCommandFinished(waitTimeInMS); // Let the hardware achieve its position.
 Sleep(1000); // Pause for a second to visually verify finished cmd.
 }

 std::cout << "Successfully finished hardware joint test.\n";
 return EcTrue;
 }
 */

size_t numJoints = 0;
cyton::hardwareInterface* hardware;
EcRealVector jointAngle;
cyton::StateType jointModifier = cyton::JointAngleInRadians
		& ~cyton::JointBaseMask;

bool goHome() {
	EcRealVector initAngles(numJoints);
	// Set all joints to their init value.
	if (!hardware->getJointStates(initAngles, cyton::InitAngle | jointModifier)) {
		std::cerr << "Unable to get init angles from getJointStates.\n";
		return EcFalse;
	}
	std::cout << "Setting all joints to init angles.\n";
	if (!hardware->setJointCommands(0.0, initAngles, jointModifier)) {
		std::cerr << "Problem setting initialize angles.\n";
		return EcFalse;
	}
}

bool printStateAndConfig() {
	// Make sure we just have a modifier parameter;
	std::cout << "Starting hardware joint test with " << numJoints
			<< " joints\n";
	EcRealVector minAngles(numJoints), initAngles(numJoints), maxAngles(
			numJoints);

	// Pull information from configuration
	std::cout << "Reading angles from configuration.\n";
	if (!hardware->getJointStates(minAngles, cyton::MinAngle | jointModifier)) {
		std::cerr << "Unable to get minimum angles from getJointStates.\n";
		return EcFalse;
	}
	else{
		for(int i=0;i<minAngles.size();i++)
			ROS_INFO("minAngle(%d) = %f",i,minAngles[i]);
	}


	if (!hardware->getJointStates(maxAngles, cyton::MaxAngle | jointModifier)) {
		std::cerr << "Unable to get maximum angles from getJointStates.\n";
		return EcFalse;
	}
	else{
		for(int i=0;i<maxAngles.size();i++)
					ROS_INFO("maxAngle(%d) = %f",i,maxAngles[i]);
	}


	if (!hardware->getJointStates(initAngles, cyton::InitAngle | jointModifier)) {
		std::cerr << "Unable to get init angles from getJointStates.\n";
		return EcFalse;
	}
	else{
		for(int i=0;i<initAngles.size();i++)
					ROS_INFO("initAngle(%d) = %f",i,initAngles[i]);
	}
}

//run example:  rostopic pub -r 5 /cmd_joints sensor_msgs/JointState '{ name: ["ww"], position: [0, 0, 0, 0, 0, 0, 2, 1], velocity: [1], effort: [0]}
//base 1 to 4.5 [home 1.5]
//shoulder 1.1 (or 1 crash) to 4.7 (4.8 crash) [home 2.9]
//elbow1  1.0 to 4.7  [home 3]
//elbow2  1.1 to 4.5  [home 3]	(1.0 relax)

void commandCallback(const sensor_msgs::JointState::ConstPtr& command) {
	const size_t numJoints = hardware->numJoints();
	if (command->position.size() != numJoints)
		ROS_ERROR("incorrect joint State Array -> correct %d, sent:%d",
				numJoints, command->position.size());
	else {
		ROS_DEBUG("okk command accepted");
		for (int i = 0; i < command->position.size(); i++) {
			jointAngle[i] = command->position[i];
		}
		hardware->setJointCommands(0, jointAngle, jointModifier);
	}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "cyton_node");
	NodeHandle node;

	EcString pluginConfig = ros::package::getPath(ROS_PACKAGE_NAME)
			+ std::string("/config/cytonConfig.xml");
	cout << pluginConfig << endl;

	//node.param("cytonConfigPath", pluginConfig, ros::package::getPath(ROS_PACKAGE_NAME)+std::string("/config/cytonConfig.xml");

	Subscriber sub_point = node.subscribe("cmd_joints", 100, commandCallback);

	// Configuration files for Cyton I hardware
	EcString pluginName = ros::package::getPath(ROS_PACKAGE_NAME)
			+ std::string("/lib/cytonPlugin.ecp");
	cout << pluginName << endl;

	hardware = new cyton::hardwareInterface(pluginName, pluginConfig);

	if ((argc > 1 && !strcmp(argv[1], "-h")) || argc > 2) {
		std::cout << "Usage: " << argv[0] << " [-h] [port]\n";
		std::cout << "\t-h : Usage information.\n\n";
		std::cout << "Will load " << pluginName << " with config file ("
				<< pluginConfig << ")\n\n";
		// Attempt to retrieve available serial ports.
		EcStringVector ports = hardware->availablePorts();
		const size_t numPorts = ports.size();
		if (numPorts) {
			std::cout << "Where port can be one of:\n";
			for (size_t ii = 0; ii < numPorts; ++ii) {
				std::cout << "\t" << ports[ii] << "\n";
			}
		}
		std::cout
				<< "\n This overrides the previously configured serial port.\n";
		return -1;
	}

	if (argc == 2) {
		hardware->setPort(argv[1]);
	}

	if (!hardware->init()) {
		std::cerr << "Problem initializing cytonHardwareInterface.\n";
		return -1;
	}

	hardware->setLowRate(EcTrue); // Run at slower rate for purposes of testing.
	numJoints = hardware->numJoints();
	jointAngle = EcRealVector(numJoints);

	/*

	 CytonController controller;
	 controller.setPluginFile("cytonPlugin.ecp");
	 controller.setSimulationFile("cyton.ecz");
	 controller.setEndEffectorIndex(0);
	 controller.setTolerance(0.003);
	 controller.setRenderingMode(EcTrue);
	 controller.setHardwareMode(EcFalse);
	 if(!controller.init())
	 {
	 std::cerr << "Problem initializing Cyton controller.\n";
	 return -1;
	 }

	 const EcReal inchToMeter=0.0254;
	 Array3Vector points;
	 points.push_back(Array3(-10, 11, -1)*inchToMeter);
	 points.push_back(Array3(-10, 11,  0)*inchToMeter);
	 points.push_back(Array3(-10, 11, -1)*inchToMeter);
	 points.push_back(Array3(-12, 11, -1)*inchToMeter);
	 points.push_back(Array3(-12, 11,  0)*inchToMeter);
	 points.push_back(Array3(-12, 11, -1)*inchToMeter);
	 points.push_back(Array3(-10,  8, -1)*inchToMeter);
	 points.push_back(Array3(-10,  8,  0)*inchToMeter);
	 points.push_back(Array3(-10,  8, -1)*inchToMeter);

	 for(EcU32 ii=0; ii<points.size(); ++ii)
	 {
	 std::cout << "Moving to point " << ii << " at: "
	 << points[ii][0] << ", "
	 << points[ii][1] << ", "
	 << points[ii][2] << std::endl;
	 if(!controller.moveTo(points[ii], 30))
	 {
	 std::cerr << "Can't reach goal.\n";
	 const Array3& actualPoint = controller.lastActualPoint();
	 std::cerr << "Current point: "
	 << actualPoint[0] << ", "
	 << actualPoint[1] << ", "
	 << actualPoint[2] << std::endl;
	 break;
	 }
	 // pause 2 seconds
	 EcSLEEPMS(2000);
	 }
	 */

	Rate loopControl(1);
	srand( time(NULL));
	while (ros::ok()) {
		ros::spinOnce();
		loopControl.sleep();
		printStateAndConfig();
	}

	//controller.reset();
	//controller.shutdown();
	hardware->shutdown();

	delete hardware;

	return 0;
}
