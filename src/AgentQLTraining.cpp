/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "AgentQLTraining.h"

AgentQLTraining::AgentQLTraining(unsigned int id, string type, Factory* factoryPtr) : Agent(id, type, factoryPtr)
{
	pesos = factoryPtr->getConstantWeights();
}

AgentQLTraining::~AgentQLTraining()
{

}

std::vector<float> AgentQLTraining::getWeights(std::vector< float > estado)
{
	return pesos;
}
