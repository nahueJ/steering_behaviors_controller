/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "AgentReactive.h"

AgentReactive::AgentReactive(unsigned int id, Factory* factoryPtr) : Agent(id, factoryPtr)
{
	pesos = factoryPtr->getConstantWeights();
}

AgentReactive::~AgentReactive()
{

}

std::vector<float> AgentReactive::getWeights(std::vector< float > estado)
{
	return pesos;
}
