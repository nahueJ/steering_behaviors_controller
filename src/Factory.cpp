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
									std::string* type,
									std::vector< std::vector<float> >* state)
{
	//busco el tipo de agente
	string agentType = cfg->lookup("simulationParams")["agentsOnSimulation"][id];
	*type = agentType;
	if (!(*type).empty()){
		cout << "the agent " << id << " is an " << *type << endl;
		//con el tipo de agente busco el elemento en el array de configuraciones de agentes correspondiente
		for (int i = 0; i < nbAgentsCfg; ++i)
		{
			string typeCfg = cfg->lookup("agents")[i]["type"];
			if (*type == typeCfg)
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
					//****************************************//
					//  Instanciación de los comportamientos  //
					//****************************************//
					for (int k = 0; k < nbBehaviorsType; ++k)
					{
						//cargar el vector de comportamiento con los que dice el array behaviorsToCreate
						behaviors->push_back(pickBehavior(behaviorsToCreate[k].c_str() , id, pre));
					}
					//*************************************************//
					//Inicializacion de la estructura de Estado General//
					//*************************************************//
					(*state).clear();
					for (int l = 0; l < behaviors->size(); ++l)
					{
						std::vector<float> auxVect = (*behaviors)[l]->getState();
						(*state).push_back(auxVect);
					}
					//******************************//
					//  Instanciación de los pesos  //
					//******************************//
					cout << "Flag1";
					*weights = pickWeights(seleccionPesos, weightsToCreate, *behaviors, state);
				}
				else{
					cout << "Missing weights for behaviors..." << endl;
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
	return auxBhPtr;
}

Weights* Factory::pickWeights(std::string weightsType, Setting& weightsVect, std::vector<SteeringBehavior*> behaviors, std::vector< std::vector<float> >* state)
{
	Weights* auxWeights;
	if (weightsType == "constW")
	{
		std::vector<float> auxW;
		for (int j = 0; j < behaviors.size(); ++j)
		{
			auxW.push_back(weightsVect[j]);
		}
		Setting& sets = cfg ->lookup("weights.constW");
		auxWeights = new Weights(auxW,&sets);
	}
	else if (weightsType == "constWOnlySeek")
	{
		std::vector<float> auxW;
		for (int j = 0; j < behaviors.size(); ++j)
		{
			auxW.push_back(weightsVect[j]);
		}
		cout << "Flag2";
		Setting& sets = cfg ->lookup("weights.constWOnlySeek");
		auxWeights = new Weights(auxW,&sets);
	}
	else if (weightsType == "constWOnlyAO")
	{
		std::vector<float> auxW;
		for (int j = 0; j < behaviors.size(); ++j)
		{
			auxW.push_back(weightsVect[j]);
		}
		Setting& sets = cfg ->lookup("weights.constWOnlyAO");
		auxWeights = new Weights(auxW,&sets);
	}
	else if (weightsType == "qvalueW")
	{
		//generar el vector con los vectores de valores posibles para generar la tabla
		// Setting& sets = cfg ->lookup("weights.qvalueW");
		//auxWeights = new Weights();
	}
	return auxWeights;
}