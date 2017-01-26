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
	//cantidad de definiciones de agentes en la conf
	nbAgentsCfg = cfg->lookup("agents").getLength();
}
	
Factory::~Factory()
{
}

int Factory::instanciateBehaviors(	unsigned int id, std::string pre, 
									std::vector<SteeringBehavior*>* behaviors, 
									Weights** weights,
									std::string* type)
{
	//busco el tipo de agente
	string agentType = 	cfg->lookup("simulationParams")["agentsOnSimulation"][id];

	if (!agentType.empty()){
		cout << "the agent " << id << " is an " << agentType << endl;
		
		//con el tipo de agente busco el elemento en el array de configuraciones de agentes correspondiente
		for (int i = 0; i < nbAgentsCfg; ++i)
		{
			string typeCfg = cfg->lookup("agents")[i]["type"];
			if (agentType == typeCfg)
			{
				//cuando encuetro el tipo correspondiente
				cout << typeCfg << " -> FOUND" << endl;
				//verifico que la cantidad de comportamientos es igual que la cantidad de pesos
				int nbBehaviorsType = cfg->lookup("agents")[i]["behaviors"].getLength();
				int nbWeightsType = cfg->lookup("agents")[i]["weights"].getLength();
				if(nbBehaviorsType == nbWeightsType)
				{
					Setting& behaviorsToCreate = cfg ->lookup("agents")[i]["behaviors"];
					Setting& weightsToCreate = cfg ->lookup("agents")[i]["weights"];
					string seleccionPesos = cfg->lookup("agents")[i]["seleccionPesos"];
					(*weights) = new Weights(seleccionPesos);
					//Instancio los comportamientos y cargo los pesos a la clase Weights correspondiente
					for (int j = 0; j < nbBehaviorsType; ++j)
					{
						//cargar el vector de comportamiento con los que dice el array behaviorsToCreate
						behaviors->push_back(pickBehavior(behaviorsToCreate[j].c_str() , id, pre));
						cout << behaviorsToCreate[j].c_str() << " instantiated" << endl;
						(*weights)->addWeight(behaviorsToCreate[j].c_str(),weightsToCreate[j]);
					}
					//si el agente se definio con aprendizaje, instancio el critico correspondiente y le paso el puntero a la clase pesos para que realize el update
					if (seleccionPesos != "no")
					{
						Setting& sets = cfg ->lookup("critics.criticQ");
						Critic* criticPtr = new Critic(id,&sets, behaviors);
						(*weights)->setCritic(criticPtr);
					}
					*type = agentType;
				}
				break;
			}
		}
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

SteeringBehavior* Factory::pickBehavior(std::string behaviorName, int id, std::string pre)
{
	SteeringBehavior* auxBhPtr;
	if (behaviorName == "seekReactive" )
	{
		Setting& sets = cfg ->lookup("seekBehaviors.seekReactive");
		auxBhPtr = new Seek (id,pre,&sets);
	}
	else if (behaviorName == "seekRL" )
	{
		Setting& sets = cfg ->lookup("seekBehaviors.seekRL");
		auxBhPtr = new Seek (id,pre,&sets);		
	}
	else if (behaviorName == "avoidObstaclesReactive")
	{
		Setting& sets = cfg ->lookup("avoidObstaclesBehaviors.avoidObstaclesReactive");
		auxBhPtr = new ObstacleAvoidance (id,pre,&sets);
	}
	else if (behaviorName == "avoidObstaclesRL")
	{
		Setting& sets = cfg ->lookup("avoidObstaclesBehaviors.avoidObstaclesRL");
		auxBhPtr = new ObstacleAvoidance (id,pre,&sets);
	}
	else if (behaviorName == "avoidObstaclesRL")
	{
		Setting& sets = cfg ->lookup("avoidObstaclesBehaviors.avoidObstaclesRL");
		auxBhPtr = new ObstacleAvoidance (id,pre,&sets);
	}
	return auxBhPtr;
}