/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "AgentReactive.h"

AgentReactive::AgentReactive(unsigned int id, string type, Factory* factoryPtr) : Agent(id, type, factoryPtr)
{
	//Pido la configuracion para el tipo de agente
	Setting* configurationPtr = factoryPtr->getTypeSetting(myType);

	//saco los pesos constantes de la conf
	Setting& ws = (*configurationPtr)["weights"];
	int nbWs = (*configurationPtr)["weights"].getLength();
	for (int k = 0; k < nbWs; ++k)
	{
		float aux = ws[k];
		pesos.push_back(aux);
	}
}

AgentReactive::~AgentReactive()
{

}

std::vector<float> AgentReactive::getWeights(std::vector<float> estado)
{
	for (std::vector<float>::iterator itb = pesos.begin(); itb != pesos.end(); ++itb)
	{
		cout << *itb << " ";
	}
	return pesos;
}
