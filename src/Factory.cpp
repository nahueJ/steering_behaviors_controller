/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "../include/Factory.h"

Factory::Factory()
{
	//Almaceno el espacio para la clase Config de libconfig++
	cfg = new Config;
	//Cargo los valores en el archivo de configuracion
	cfg->readFile(CONFIGFILE);
	//inicializo la variable
	nbAgents = cfg->lookup("simulationParams")["agentsOnSimulation"].getLength();	
}
	
Factory::~Factory()
{
}

int Factory::instanciateBehaviors(unsigned int id, std::string pre, std::vector<SteeringBehavior*> behaviors, std::vector<float> weights)
{
	// Setting& params = cfg->lookup("simulationParams");
	cout << "aqui" << endl;
	//busco el tipo de agente
	string agentType = 	cfg->lookup("simulationParams")["agentsOnSimulation"][id];
	//con el tipo de agente busco el array de arreglos correspondientes
	// y la lista de pesos
	SteeringBehavior* auxBhPtr;
	if (!agentType.empty())
	{
		cout << "the agent " << id << " is an " << agentType << endl;
		//Instanciate Seek
		auxBhPtr = new Seek (id,pre,cfg);
		behaviors.push_back(auxBhPtr);
		//Instanciate ObstacleAvoidance
		auxBhPtr = new ObstacleAvoidance (id,pre,cfg);
		behaviors.push_back(auxBhPtr);
		return 1;
	}
	else{
		cout << "Cant find agent " << id << " definition" << endl;
		return 0;
	}

}

int Factory::getAgents()
{
	return nbAgents;
}