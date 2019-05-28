#include "sml_Client.h"
#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include <ros/package.h>

using namespace std;
using namespace sml;

float sonars_output[8];

void mapErraticMessageToFloat(const std_msgs::String::ConstPtr& msg){

}

int main(int argc, char **argv) {

	Kernel* pKernel = Kernel::CreateKernelInNewThread();

	if (pKernel->HadError()) {
		cout << pKernel->GetLastErrorDescription() << endl;
		return -1;
	}

	sml::Agent* pAgent = pKernel->CreateAgent("test2");

	if (pKernel->HadError()) {
		cout << pKernel->GetLastErrorDescription() << endl;
		return -1;
	}

	std::string path = ros::package::getPath(ROS_PACKAGE_NAME)+ std::string("/example/reglas-movimiento.soar");
	pAgent->LoadProductions(path.c_str());

	if (pAgent->HadError()) {
		cout << pAgent->GetLastErrorDescription() << endl;
		return -1;
	}

	bool exit = false;

	ros::init(argc, argv, "listener");
	// suscripcion al cmd_vel
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("sonar", 1000, mapErraticMessageToFloat);

	// while(!exit){

	ros::spin();

	int i;
	float value;

	//Obtenemos el identificador el input-link
	Identifier* pInputLink = pAgent->GetInputLink();

	Identifier* pID = pAgent->CreateIdWME(pInputLink, "sonars-output");
	FloatElement* pWME0 = pAgent->CreateFloatWME(pID, "sonarNo0", sonars_output[0]);
	FloatElement* pWME1 = pAgent->CreateFloatWME(pID, "sonarNo1", sonars_output[1]);
	FloatElement* pWME2 = pAgent->CreateFloatWME(pID, "sonarNo2", sonars_output[2]);
	FloatElement* pWME3 = pAgent->CreateFloatWME(pID, "sonarNo3", sonars_output[3]);
	FloatElement* pWME4 = pAgent->CreateFloatWME(pID, "sonarNo4", sonars_output[4]);
	FloatElement* pWME5 = pAgent->CreateFloatWME(pID, "sonarNo5", sonars_output[5]);
	FloatElement* pWME6 = pAgent->CreateFloatWME(pID, "sonarNo6", sonars_output[6]);
	FloatElement* pWME7 = pAgent->CreateFloatWME(pID, "sonarNo7", sonars_output[7]);

	pAgent->Commit();

	pAgent->RunSelfTilOutput();

	cout << "El agente ha generado comandos de salida" << endl;

	pAgent->Commands();
	int numberCommands = pAgent->GetNumberCommands();


	cout << "Numero de comandos recibidos del agente: " << numberCommands << endl;

	for (int i = 0; i < numberCommands; i++) {

		Identifier * pCommand = pAgent->GetCommand(i);

		std::string name = pCommand->GetCommandName();

		if(name == "move"){
			std::string move_dir = pCommand->GetParameterValue("move_dir");
		}

		// Actualizar aquí el entorno para reflejar el comando del agente

		if(move_dir == "north"){
			cout << "Mover el agente hacia el norte" << endl;
		}else if(move_dir == "north-east"){
			cout << "Mover el agente hacia el noreste" << endl;
		}else if(move_dir == "east"){
			cout << "Mover el agente hacia el este" << endl;
		}else if(move_dir == "south-east"){
			cout << "Mover el agente hacia el sureste" << endl;
		}else if(move_dir == "south"){
			cout << "Mover el agente hacia el sur" << endl;
		}else if(move_dir == "south-west"){
			cout << "Mover el agente hacia el suroeste" << endl;
		}else if(move_dir == "west"){
			cout << "Mover el agente hacia el oeste" << endl;
		}else if(move_dir == "north-west"){
			cout << "Mover el agente hacia el noroeste" << endl;
		}else{
			cout << "ERROR: comando de movimiento incorrecto" << endl;
		}


		pCommand->AddStatusComplete();

	}

	pKernel->CheckForIncomingCommands();


	pKernel->Shutdown();
	delete pKernel;

	return 0;

} // fin main

