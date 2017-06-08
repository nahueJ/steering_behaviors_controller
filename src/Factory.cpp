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
}

Factory::~Factory()
{
}

int Factory::instanciateBehaviors(	unsigned int id, std::string pre,
									std::vector<SteeringBehavior*>* behaviors,
									std::string type)
{
	//busco el tipo de agente
	agentType << "agents." << type;

	Setting& behaviorsToCreate = cfg ->lookup(agentType.str())["behaviors"];
	int nbBehaviorsType = cfg->lookup(agentType.str())["behaviors"].getLength();

	for (int k = 0; k < nbBehaviorsType; ++k)
	{
		//cargar el vector de comportamiento con los que dice el array behaviorsToCreate
		behaviors->push_back(pickBehavior(behaviorsToCreate[k].c_str() , id, pre));
	}

	return 1;
}


SteeringBehavior* Factory::pickBehavior(std::string behaviorName, int id, std::string pre)
{
	std::stringstream bhType;
	bhType << "behaviors." << behaviorName;
	Setting& sets = cfg ->lookup(bhType.str());

	SteeringBehavior* auxBhPtr;

	if (behaviorName == "seek" )
	{
		auxBhPtr = new Seek (id,pre,&sets);
	}
	else if (behaviorName == "obstacleAvoidance")
	{
		auxBhPtr = new ObstacleAvoidance (id,pre,&sets);
	}

	return auxBhPtr;
}

Setting* Factory::getTypeSetting(std::string type)
{
	std::stringstream agType;
	agType << "agents." << type;
	Setting& sets = cfg ->lookup(agType.str());
	return &sets;
}
/*Weights* Factory::pickWeights(std::string weightsType, Setting& weightsVect, std::vector<SteeringBehavior*> behaviors, std::vector< std::vector<float> >* state)
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
		Setting& sets = cfg ->lookup("weights.qvalueW");
		//Genero un vector con los posibles valores para cada elemento del vector de estado, asi es posible generar las combinaciones para la qTable
		std::vector< std::vector< std::vector<float> > > statePosibilities;
		for (int i = 0; i < (*state).size(); ++i)
		{
			std::vector< std::vector<float> > auxv;
			for (int j = 0; j < (*state)[i].size(); ++j)
			{
				auxv.push_back(behaviors[i]->getPosibleValues());
			}
			statePosibilities.push_back(auxv);
		}
		auxWeights = new Weights(statePosibilities, &sets);
	}
	else if (weightsType == "constQvalueW")
	{
		//generar el vector con los vectores de valores posibles para generar la tabla
		Setting& sets = cfg ->lookup("weights.constQvalueW");

		auxWeights = new Weights(&sets);
	}
	else if (weightsType == "qvalueWFile")
	{
		//generar el vector con los vectores de valores posibles para generar la tabla
		Setting& sets = cfg ->lookup("weights.qvalueWFile");

		auxWeights = new Weights(&sets);
	}
	return auxWeights;
}*/
