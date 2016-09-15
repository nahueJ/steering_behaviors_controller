/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _FACTORY_H
#define _FACTORY_H

#include "SteeringBehavior.h"

#include "ObstacleAvoidance.h"
#include "Seek.h"

#include <libconfig.h>

#include <string>
#include <vector>
#include <sstream>

class Factory {
public: 

	/**
	 * @param unsigned int id
	 */
	Factory(config_t* configurationPtr);

	//Factory(i,behaviors[i], FactoryPtr);
	
	~Factory();

	std::vector<SteeringBehavior*> instanciateBehaviors(unsigned int id, std::string pre);

private: 

	std::vector<SteeringBehavior*>* vectorPtr;

	config_t config;

	int nbAgents;

	string agentsType[];
};

#endif //_FACTORY_H