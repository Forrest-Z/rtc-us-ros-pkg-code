// Normalmente solo se necesita este header
#include "sml_Client.h"
#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;
using namespace sml;

void onProduction(smlProductionEventId id, void* pUserData, Agent* pAgent, char const* pProdName, char const* pInstantion){
	cout << "Se ha disparado la producción: " << pProdName << endl;
}


int main() {

	// Crea una instancia del kernel de Soar en nuestro proceso
	Kernel* pKernel = Kernel::CreateKernelInNewThread();

	// Comprueba que nada haya ido mal. Siempre devolveremos un objeto de tipo kernel
	// incluso si ha habido errores y tenemos que abortar
	if (pKernel->HadError()) {
		cout << pKernel->GetLastErrorDescription() << endl;
		return -1;
	}

	// Crea un nuevo agente Soar llamado "test"
	// NOTA: no eliminamos el puntero al agente. Le pertenece al kernel
	sml::Agent* pAgent = pKernel->CreateAgent("test");

	// Comprueba que nada haya ido mal
	// NOTA: ningún agente se crea si ha habido errores, así que tenemos que
	// comprobar los errores a través del objeto kernel
	if (pKernel->HadError()) {
		cout << pKernel->GetLastErrorDescription() << endl;
		return -1;
	}

	// Carga algunas reglas
	std::string path=ros::package::getPath(ROS_PACKAGE_NAME)+ std::string("/example/testsml.soar");
	pAgent->LoadProductions(path.c_str());

	if (pAgent->HadError()) {
		cout << pAgent->GetLastErrorDescription() << endl;
		return -1;
	}
	/*

	Identifier* pInputLink = pAgent->GetInputLink();

	// Crea los elem. (I3 ^plane P1) (P1 ^type Boeing747 ^speed 200 ^direction 50.5) en
	// el input-link.
	Identifier* pID = pAgent->CreateIdWME(pInputLink, "plane");
	StringElement* pWME1 = pAgent->CreateStringWME(pID, "type", "Boeing747");
	IntElement* pWME2 = pAgent->CreateIntWME(pID, "speed", 200);
	FloatElement* pWME3 = pAgent->CreateFloatWME(pID, "direction", 50.5);

	// Envía los cambios a la memoria de trabajo de SOAR
	// En 8.6.2 esta llamada es opcional, ya que los cambios se envían automáticamente
	pAgent->Commit();



	// Ejecutar SOAR para dos decisiones
	pAgent->RunSelf(2);


	// Cambia (P1 ^speed) a 300 y envía el cambio a SOAR
	pAgent->Update(pWME2, 300);
	pAgent->Commit();

	*/

	// Ejecuta SOAR hasta que genere alguna salida o hayan pasado 15 ciclos de decisión
	// (El caso más típico es ejecutar para una decisión en vez de hasta que haya alguna salida)

	int callbackp = pAgent->RegisterForProductionEvent(smlEVENT_AFTER_PRODUCTION_FIRED, onProduction, NULL) ;

	 pAgent->RunSelfTilOutput();
	//pAgent->RunSelf(2);

	cout << "El agente ha generado comandos de salida" << endl;

	// Recorre todos los comandos que hemos recibido (si los hay) desde la última vez que ejecutamos SOAR.
	pAgent->Commands();
	int numberCommands = pAgent->GetNumberCommands();


	cout << "Numero de comandos recibidos del agente: " << numberCommands << endl;

	for (int i = 0; i < numberCommands; i++) {

		Identifier * pCommand = pAgent->GetCommand(i);

		std::string name = pCommand->GetCommandName();

		std::string mensaje = pCommand->GetParameterValue("mensaje");

		// Actualizar aquí el entorno para reflejar el comando del agente

		if(mensaje=="hola-mundo"){
			cout << "Hola Mundo!" << endl;
		}

		// Luego marcar el comando como completado
		pCommand->AddStatusComplete();

		// O también se podría hacer lo mismo manualmente así:
		// pAgent->CreateStringWME(pCommand, "status", "complete") ;
	}

	// Mira si alguien (p.ej. un depurador) ha enviado comandos a SOAR
	// Si no llamamos a este metodo periodicamente, las conexiones remotas serán ignoradas si
	// elegimos el método "CreateKernelInCurrentThread".
	pKernel->CheckForIncomingCommands();

	// Creamos un comando SOAR de ejemplo
	std::string cmd = "excise --all";

	// Ejecuta el comando
	char const* pResult = pKernel->ExecuteCommandLine(cmd.c_str(),
			pAgent->GetAgentName());


	// Apagado y limpieza
	pKernel->Shutdown(); // Elimina todos los agentes (a menos que se use una conexión remota)
	delete pKernel; // Elimina el kernel

	return 0;

} // fin main

