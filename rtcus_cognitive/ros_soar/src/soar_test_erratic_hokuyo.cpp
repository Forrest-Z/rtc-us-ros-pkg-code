#include "sml_Client.h"
#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <erratic_player/Range.h>
#include <erratic_player/RangeArray.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace sml;

// Erratic robot simulation parameters:

#define LINEAR_SPEED 0.33 	   /* Robot linear speed in metres/second. Maximum in Gazebo: 0.35 m/s */
#define ANGULAR_SPEED_RATIO 0.6   /* Angular speed tune: above 1 more angular speed. Below 1 
				      less angular speed */
#define LASERS_PER_READING 13	/* Width scan area of each 'sonar'. The more narrow, the more
				   probability to cross a door. The wider individual sonar scan area,
				   the higher probability to avoid any obstacle */

#define SOAR_FILE_PATH "/src/sp-look-for-walls-09-metres.soar" /* This constant set the .soar file to be 
						        loaded. Two demo soar files are available:
						        'sp-simple-reactive-movement.soar' and
						        'sp-look-for-walls.soar' */

//#define SOAR_FILE_PATH "/src/sp-simple-reactive-movement.soar"				  



float sonars_output[8];

// Radians to degrees
float r2d = 180.0/3.14159;

// Prints any soar production after firing
void onProduction(smlProductionEventId id, void* pUserData, Agent* pAgent, char const* pProdName, char const* pInstantion){
	ROS_INFO( "Production fired: %s", pProdName);
}


// Filter the laser readings. It takes a laser reading position and returns the minimum of its neighbours (number_readings) values.
float filter_laser_readings (const sensor_msgs::LaserScanConstPtr& msg, int laser_pos, int number_readings) {
	int i, valid_readings=0;
	float minimun=5; 	/* 'minimun' is the initial distance value for each sonar. 
			    	    Laser scan has 4 metres length area */
	float threshold = 0.02; /* 'threshold' indicates when a laser reading will be considered 
				   valid. Hokuyo Laser scan returns 0 or almost 0 when an obstacle is 
				   far and/or it can not detected. This does not happend in a the 
				   Gazebo simulation where conditions were ideal. */
	//
	for (i=laser_pos; i < laser_pos + number_readings/2; i++) {
		if (msg->ranges[i] > 0 && msg->ranges[i] <= 4) {
			valid_readings++;
			if ((msg->ranges[i]) < minimun)
				minimun = msg->ranges[i];
		}
	}
	//
	for (i=laser_pos; i >= laser_pos - number_readings/2; i--) {
		if (msg->ranges[i] > 0 && msg->ranges[i] <= 4) {
			valid_readings++;
			if ((msg->ranges[i]) < minimun) {
				minimun = msg->ranges[i];
			}
		}
	}
	//
	if (valid_readings > 0 && minimun >= threshold) {
		return (minimun);
	} else {
		return 4.0;
	}
}
/**/


// Stores few selected range inputs from the erratic into the sonars_output array
void mapErraticMessageToFloat(const sensor_msgs::LaserScanConstPtr& msg){
	// Hokuyo URG-04LX has 240º scanning area, whreas in Gazebo lase has 270º scanning area.
	int offset=-3; // offset: in Gazebo msg->ranges.size() == 683 and in URG-04LX == 680
	sonars_output[0] = filter_laser_readings (msg, 114+offset, LASERS_PER_READING);
	sonars_output[1] = filter_laser_readings (msg, 190+offset, LASERS_PER_READING);
	sonars_output[2] = filter_laser_readings (msg, 266+offset, LASERS_PER_READING);
	sonars_output[3] = filter_laser_readings (msg, 340+offset, LASERS_PER_READING);
	sonars_output[4] = filter_laser_readings (msg, 342+offset, LASERS_PER_READING);
	sonars_output[5] = filter_laser_readings (msg, 417+offset, LASERS_PER_READING);
	sonars_output[6] = filter_laser_readings (msg, 493+offset, LASERS_PER_READING);
	sonars_output[7] = filter_laser_readings (msg, 568+offset, LASERS_PER_READING);


	// Monitor laser readings with 'rxconsole' command in ROS:
	//ROS_INFO("Laser ranges (%d): \n 0: %.2f \n 1:%.2f \n 2:%.2f \n 3:%.2f \n 4:%.2f \n 5:%.2f \n 6:%.2f \n 7:%.2f", (msg->ranges.size()), sonars_output[0], sonars_output[1], sonars_output[2], sonars_output[3] ,sonars_output[4] ,sonars_output[5] ,sonars_output[6] ,sonars_output[7]);
}


int main(int argc, char **argv) {

	// Soar kernel and agent initialization
	Kernel* pKernel = Kernel::CreateKernelInNewThread();

	if (pKernel->HadError()) {
		cout << pKernel->GetLastErrorDescription() << endl;
		return -1;
	}

	sml::Agent* pAgent = pKernel->CreateAgent("wandering-agent");

	if (pKernel->HadError()) {
		cout << pKernel->GetLastErrorDescription() << endl;
		return -1;
	}

	// Soar rules path and load
	std::string path = ros::package::getPath(ROS_PACKAGE_NAME)+ std::string(SOAR_FILE_PATH);


	pAgent->LoadProductions(path.c_str());

	if (pAgent->HadError()) {
		cout << pAgent->GetLastErrorDescription() << endl;
		return -1;
	}

	int callbackp = pAgent->RegisterForProductionEvent(smlEVENT_AFTER_PRODUCTION_FIRED, onProduction, NULL) ;

	ros::init(argc, argv, "wander");
	ros::NodeHandle n;

	// cmd_vel subscription
	ros::Subscriber sub = n.subscribe("laser", 1, mapErraticMessageToFloat);

	// Movement commands publisher
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("commands",1);

	// set up looping timer at 10 Hz
	ros::Rate rate(10);

	bool init = true;

	// twist (movement) message
	geometry_msgs::Twist twist;

	// linear speed (m/s)
	double xvel = LINEAR_SPEED; 

	twist.linear.x = xvel;

	// angular speed (radians/s)
	double thvel;

	Identifier* pInputLink;

	Identifier* pID;
	FloatElement* pWME0;
	FloatElement* pWME1;
	FloatElement* pWME2;
	FloatElement* pWME3;
	FloatElement* pWME4;
	FloatElement* pWME5;
	FloatElement* pWME6;
	FloatElement* pWME7;

	while(ros::ok()){

		if(init){
			//input-link identificator
			pInputLink = pAgent->GetInputLink();

			pID = pAgent->CreateIdWME(pInputLink, "sonars-output");
			pWME0 = pAgent->CreateFloatWME(pID, "sonarNo0", sonars_output[0]);
			pWME1 = pAgent->CreateFloatWME(pID, "sonarNo1", sonars_output[1]);
			pWME2 = pAgent->CreateFloatWME(pID, "sonarNo2", sonars_output[2]);
			pWME3 = pAgent->CreateFloatWME(pID, "sonarNo3", sonars_output[3]);
			pWME4 = pAgent->CreateFloatWME(pID, "sonarNo4", sonars_output[4]);
			pWME5 = pAgent->CreateFloatWME(pID, "sonarNo5", sonars_output[5]);
			pWME6 = pAgent->CreateFloatWME(pID, "sonarNo6", sonars_output[6]);
			pWME7 = pAgent->CreateFloatWME(pID, "sonarNo7", sonars_output[7]);

			init = false;

		}else{

			pAgent->Update(pWME0, sonars_output[0]);
			pAgent->Update(pWME1, sonars_output[1]);
			pAgent->Update(pWME2, sonars_output[2]);
			pAgent->Update(pWME3, sonars_output[3]);
			pAgent->Update(pWME4, sonars_output[4]);
			pAgent->Update(pWME5, sonars_output[5]);
			pAgent->Update(pWME6, sonars_output[6]);
			pAgent->Update(pWME7, sonars_output[7]);

		}

		pAgent->Commit();

		pAgent->RunSelfTilOutput();

		pAgent->Commands();
		int numberCommands = pAgent->GetNumberCommands();

		// Debug:
		// cout << "Number of commands received: " << numberCommands << endl;

		for (int i = 0; i < numberCommands; i++) {

			std::string move_dir;

			Identifier * pCommand = pAgent->GetCommand(i);

			std::string name = pCommand->GetCommandName();

			if(name == "move"){
				move_dir = pCommand->GetParameterValue("move_dir");
	
				// Direction treatment
				
				if(move_dir == "zero"){
					// ROS_INFO("Move the agent to direction 0");
					thvel = -90/r2d;
				}else if(move_dir == "one"){
					// ROS_INFO("Move the agent to direction 1");
					thvel = -60/r2d;
				}else if(move_dir == "two"){
					// ROS_INFO("Move the agent to direction 2");
					thvel = -30/r2d;
				}else if(move_dir == "three"){
					// ROS_INFO("Move the agent to direction 3");
					thvel = 0;
				}else if(move_dir == "four"){
					// ROS_INFO("Move the agent to direction 4");
					thvel = 0;
				}else if(move_dir == "five"){
					// ROS_INFO("Move the agent to direction 5");
					thvel = 30/r2d;
				}else if(move_dir == "six"){
					// ROS_INFO("Move the agent to direction 6");
					thvel = 60/r2d;
				}else if(move_dir == "seven"){
					// ROS_INFO("Move the agent to direction 7");
					thvel = 90/r2d;
				}else{
					// ROS_INFO("ERROR: invalid command!");
				}
	
				pCommand->AddStatusComplete();
	
				// Set the angle into the movement message
				twist.angular.z = thvel*ANGULAR_SPEED_RATIO;
			
			}
		}

		pKernel->CheckForIncomingCommands();

		// Movement command publication
		pub.publish(twist);

	    ros::spinOnce();
	    rate.sleep();

	}

	// Soar agent shutdown
	pKernel->Shutdown();
	delete pKernel;

	return 0;

} // end main

