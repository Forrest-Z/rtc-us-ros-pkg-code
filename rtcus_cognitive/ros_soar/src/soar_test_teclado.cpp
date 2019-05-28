#include "sml_Client.h"
#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;
using namespace sml;

float sonars_output[8];

int main(int argc, char **argv) {

	Kernel* pKernel = Kernel::CreateKernelInNewThread();

	if (pKernel->HadError()) {
		cout << pKernel->GetLastErrorDescription() << endl;
		return -1;
	}

	sml::Agent* pAgent = pKernel->CreateAgent("test_agent");

	if (pKernel->HadError()) {
		cout << pKernel->GetLastErrorDescription() << endl;
		return -1;
	}

	std::string path = ros::package::getPath(ROS_PACKAGE_NAME)+ std::string("/src/reglas-movimiento.soar");
	pAgent->LoadProductions(path.c_str());

	if (pAgent->HadError()) {
		cout << pAgent->GetLastErrorDescription() << endl;
		return -1;
	}

	bool init = true;

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

	bool salir = false;
	string input = "";
	float valor;
	int i;

	while(!salir){

		cout << "Introduzca 8 valores para los sensores, o un nro negativo para salir" << endl;

		for(i=0;i<8 && !salir;i++){
			while (true) {
			   cout << "Introduzca un numero para el para el sonar " << i << ": ";
			   getline(cin, input);
			   stringstream myStream(input);
			   if (myStream >> valor){
				 if(valor < 0) salir = true;
				 sonars_output[i] = valor;
			     break;
			   }
			   cout << "Valor invalido" << endl;
			 }
		}

		cout << "Valores de los sonars leidos: " << endl;
		for(i=0;i<8;i++){
			cout << "Valor " << i << ": " << sonars_output[i] << endl;
		}

		if(init){
			//Obtenemos el identificador del input-link
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

		cout << "El agente ha generado comandos de salida" << endl;

		pAgent->Commands();
		int numberCommands = pAgent->GetNumberCommands();


		cout << "Numero de comandos recibidos del agente: " << numberCommands << endl;

		for (int i = 0; i < numberCommands; i++) {

			std::string move_dir;

			Identifier * pCommand = pAgent->GetCommand(i);

			std::string name = pCommand->GetCommandName();

			if(name == "move"){
				move_dir = pCommand->GetParameterValue("move_dir");
			}

			// Actualizar aquí el entorno para reflejar el comando del agente

			if(move_dir == "zero"){
				cout << "Mover el agente hacia la dir. 0" << endl;
			}else if(move_dir == "one"){
				cout << "Mover el agente hacia la dir. 1" << endl;
			}else if(move_dir == "two"){
				cout << "Mover el agente hacia la dir. 2" << endl;
			}else if(move_dir == "three"){
				cout << "Mover el agente hacia la dir. 3" << endl;
			}else if(move_dir == "four"){
				cout << "Mover el agente hacia la dir. 4" << endl;
			}else if(move_dir == "five"){
				cout << "Mover el agente hacia la dir. 5" << endl;
			}else if(move_dir == "six"){
				cout << "Mover el agente hacia la dir. 6" << endl;
			}else if(move_dir == "seven"){
				cout << "Mover el agente hacia la dir. 7" << endl;
			}else{
				cout << "ERROR: comando de movimiento incorrecto" << endl;
			}

			pCommand->AddStatusComplete();

		}


		pKernel->CheckForIncomingCommands();

	}

	pKernel->Shutdown();
	delete pKernel;

	return 0;

} // fin main


